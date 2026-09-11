#include "mission_core/mission_tree.hpp"

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <tinyxml2.h>

namespace mission_core
{
namespace
{

using tinyxml2::XMLElement;

std::string attribute(const XMLElement & element, const char * name, const std::string & fallback = {})
{
  const char * value = element.Attribute(name);
  return value == nullptr ? fallback : value;
}

std::string required_attribute(const XMLElement & element, const char * name)
{
  const std::string value = attribute(element, name);
  if (value.empty()) {
    throw std::runtime_error(
            "element <" + std::string(element.Name()) + "> requires attribute " + name);
  }
  return value;
}

double required_seconds(const XMLElement & element)
{
  const auto value = required_attribute(element, "seconds");
  double seconds = 0.0;
  if (!parse_double(value, seconds) || seconds < 0.0) {
    throw std::runtime_error("invalid seconds on <" + std::string(element.Name()) + ">");
  }
  return seconds;
}

}  // namespace

struct MissionTree::Impl
{
  struct Context
  {
    const WorldState & world;
    const CapabilityState & capabilities;
    const std::vector<MissionEvent> & events;
    IActionExecutor & executor;
    MissionRuntime & runtime;
    std::vector<MissionEvent> emitted_events;
    std::string error;

    bool has_event(const std::string & name) const
    {
      return std::any_of(events.begin(), events.end(), [&name](const MissionEvent & event) {
        return event.name == name;
      });
    }

    void set_node(const std::string & node_name)
    {
      runtime.active_node = node_name;
    }

    void reject(std::string reason)
    {
      if (error.empty()) {
        error = std::move(reason);
      }
      runtime.failure_reason = error;
    }
  };

  struct Node
  {
    explicit Node(std::string node_name)
    : name(std::move(node_name)) {}
    virtual ~Node() = default;
    virtual NodeStatus tick(Context & context) = 0;
    virtual void reset() = 0;

    std::string name;
  };

  struct SequenceNode final : Node
  {
    SequenceNode(std::string node_name, bool reactive)
    : Node(std::move(node_name)), reactive_(reactive) {}

    NodeStatus tick(Context & context) override
    {
      context.set_node(name);
      if (reactive_) {
        index_ = 0U;
      }
      while (index_ < children_.size()) {
        const auto status = children_[index_]->tick(context);
        if (status == NodeStatus::running) {
          return status;
        }
        if (status == NodeStatus::failure) {
          return status;
        }
        ++index_;
      }
      return NodeStatus::success;
    }

    void reset() override
    {
      index_ = 0U;
      for (auto & child : children_) {
        child->reset();
      }
    }

    std::vector<std::unique_ptr<Node>> children_;
    std::size_t index_{0U};
    bool reactive_{false};
  };

  struct FallbackNode final : Node
  {
    FallbackNode(std::string node_name, bool reactive)
    : Node(std::move(node_name)), reactive_(reactive) {}

    NodeStatus tick(Context & context) override
    {
      context.set_node(name);
      if (reactive_) {
        index_ = 0U;
      }
      while (index_ < children_.size()) {
        const auto status = children_[index_]->tick(context);
        if (status == NodeStatus::running || status == NodeStatus::success) {
          return status;
        }
        ++index_;
      }
      return NodeStatus::failure;
    }

    void reset() override
    {
      index_ = 0U;
      for (auto & child : children_) {
        child->reset();
      }
    }

    std::vector<std::unique_ptr<Node>> children_;
    std::size_t index_{0U};
    bool reactive_{false};
  };

  struct WaitNode final : Node
  {
    WaitNode(std::string node_name, double seconds)
    : Node(std::move(node_name)), seconds_(seconds) {}

    NodeStatus tick(Context & context) override
    {
      context.set_node(name);
      if (!started_at_) {
        started_at_ = context.world.now_s;
      }
      if (context.world.now_s < *started_at_) {
        context.reject("clock moved backwards while waiting at node " + name);
        return NodeStatus::failure;
      }
      return context.world.now_s - *started_at_ >= seconds_ ?
        NodeStatus::success : NodeStatus::running;
    }

    void reset() override {started_at_.reset();}

    double seconds_{0.0};
    std::optional<double> started_at_{};
  };

  struct RequireCapabilityNode final : Node
  {
    RequireCapabilityNode(std::string node_name, std::string capability)
    : Node(std::move(node_name)), capability_(std::move(capability)) {}

    NodeStatus tick(Context & context) override
    {
      context.set_node(name);
      if (context.capabilities.available(capability_)) {
        return NodeStatus::success;
      }
      const auto availability = context.capabilities.availability(capability_);
      if (availability == CapabilityAvailability::unsupported) {
        context.reject(
          "capability " + capability_ + " is unsupported" +
          (context.capabilities.reason(capability_).empty() ? "" :
          ": " + context.capabilities.reason(capability_)));
        return NodeStatus::failure;
      }
      return NodeStatus::running;
    }

    void reset() override {}

    std::string capability_;
  };

  struct ExecuteActionNode final : Node
  {
    ExecuteActionNode(
      std::string node_name, std::string action_id, std::string capability,
      std::map<std::string, std::string> parameters)
    : Node(std::move(node_name)), action_id_(std::move(action_id)),
      capability_(std::move(capability)), parameters_(std::move(parameters)) {}

    NodeStatus tick(Context & context) override
    {
      context.set_node(name);
      if (!submitted_) {
        intent_ = ActionIntent{
          context.runtime.mission_id, action_id_, capability_, parameters_};
        std::string reason;
        if (!context.executor.submit(intent_, context.world.now_s, reason)) {
          context.reject(
            reason.empty() ? "executor rejected action " + action_id_ : reason);
          return NodeStatus::failure;
        }
        submitted_ = true;
        context.runtime.active_action_id = action_id_;
        context.runtime.active_capability = capability_;
      }

      const auto feedback = context.executor.feedback(action_id_);
      switch (feedback.status) {
        case ActionStatus::accepted:
        case ActionStatus::running:
        case ActionStatus::paused:
        case ActionStatus::idle:
          return NodeStatus::running;
        case ActionStatus::succeeded:
          if (context.runtime.active_action_id == action_id_) {
            context.runtime.active_action_id.clear();
            context.runtime.active_capability.clear();
          }
          return NodeStatus::success;
        case ActionStatus::failed:
        case ActionStatus::cancelled:
          context.reject(
            feedback.reason.empty() ? "action failed: " + action_id_ : feedback.reason);
          return NodeStatus::failure;
      }
      return NodeStatus::failure;
    }

    void reset() override
    {
      submitted_ = false;
      intent_ = {};
    }

    std::string action_id_;
    std::string capability_;
    std::map<std::string, std::string> parameters_;
    bool submitted_{false};
    ActionIntent intent_{};
  };

  struct WaitEventNode final : Node
  {
    WaitEventNode(std::string node_name, std::string event_name)
    : Node(std::move(node_name)), event_name_(std::move(event_name)) {}

    NodeStatus tick(Context & context) override
    {
      context.set_node(name);
      return context.has_event(event_name_) ? NodeStatus::success : NodeStatus::running;
    }

    void reset() override {}
    std::string event_name_;
  };

  struct EmitEventNode final : Node
  {
    EmitEventNode(std::string node_name, std::string event_name, std::string payload)
    : Node(std::move(node_name)), event_name_(std::move(event_name)), payload_(std::move(payload)) {}

    NodeStatus tick(Context & context) override
    {
      context.set_node(name);
      context.emitted_events.push_back({event_name_, payload_, context.world.now_s});
      return NodeStatus::success;
    }

    void reset() override {}
    std::string event_name_;
    std::string payload_;
  };

  std::unique_ptr<Node> parse_node(const XMLElement & element, const std::string & path)
  {
    const std::string tag = element.Name() == nullptr ? std::string{} : element.Name();
    const std::string node_name = attribute(element, "name", attribute(element, "id", tag));
    if (tag == "Sequence" || tag == "ReactiveSequence") {
      auto node = std::make_unique<SequenceNode>(node_name, tag == "ReactiveSequence");
      for (const auto * child = element.FirstChildElement(); child; child = child->NextSiblingElement()) {
        node->children_.push_back(parse_node(*child, path + "/" + node_name));
      }
      if (node->children_.empty()) {
        throw std::runtime_error("empty sequence at " + path + "/" + node_name);
      }
      return node;
    }
    if (tag == "Fallback" || tag == "ReactiveFallback") {
      auto node = std::make_unique<FallbackNode>(node_name, tag == "ReactiveFallback");
      for (const auto * child = element.FirstChildElement(); child; child = child->NextSiblingElement()) {
        node->children_.push_back(parse_node(*child, path + "/" + node_name));
      }
      if (node->children_.empty()) {
        throw std::runtime_error("empty fallback at " + path + "/" + node_name);
      }
      return node;
    }
    if (tag == "Wait") {
      return std::make_unique<WaitNode>(node_name, required_seconds(element));
    }
    if (tag == "RequireCapability") {
      const auto capability = required_attribute(element, "capability");
      declared_capabilities_.push_back(capability);
      return std::make_unique<RequireCapabilityNode>(node_name, capability);
    }
    if (tag == "WaitEvent") {
      return std::make_unique<WaitEventNode>(node_name, required_attribute(element, "event"));
    }
    if (tag == "EmitEvent") {
      return std::make_unique<EmitEventNode>(
        node_name, required_attribute(element, "event"), attribute(element, "payload"));
    }
    if (tag == "ExecuteAction") {
      const std::string capability = required_attribute(element, "capability");
      declared_capabilities_.push_back(capability);
      const std::string action_id = attribute(element, "id", node_name);
      std::map<std::string, std::string> parameters;
      for (const auto * item = element.FirstAttribute(); item; item = item->Next()) {
        const std::string key = item->Name();
        if (key != "name" && key != "id" && key != "capability") {
          parameters.emplace(key, item->Value());
        }
      }
      return std::make_unique<ExecuteActionNode>(
        node_name, action_id, capability, std::move(parameters));
    }
    throw std::runtime_error("unknown mission node <" + tag + "> at " + path);
  }

  explicit Impl(const std::string & xml_path, std::string mission_id)
  : mission_id_(std::move(mission_id))
  {
    if (xml_path.empty()) {
      throw std::invalid_argument("mission XML path must not be empty");
    }
    if (mission_id_.empty()) {
      throw std::invalid_argument("mission id must not be empty");
    }

    tinyxml2::XMLDocument document;
    if (document.LoadFile(xml_path.c_str()) != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error(
              "failed to load mission XML: " + xml_path + ": " + document.ErrorStr());
    }
    const auto * root = document.FirstChildElement("root");
    if (root == nullptr) {
      throw std::runtime_error("mission XML requires a <root> element");
    }
    const std::string requested_tree = attribute(*root, "main_tree_to_execute", "MainTree");
    const XMLElement * tree_element = nullptr;
    for (const auto * candidate = root->FirstChildElement("BehaviorTree"); candidate;
      candidate = candidate->NextSiblingElement("BehaviorTree"))
    {
      if (attribute(*candidate, "ID") == requested_tree) {
        tree_element = candidate;
        break;
      }
    }
    if (tree_element == nullptr) {
      throw std::runtime_error("mission XML does not contain tree " + requested_tree);
    }
    const auto * first_node = tree_element->FirstChildElement();
    if (first_node == nullptr || first_node->NextSiblingElement() != nullptr) {
      throw std::runtime_error("BehaviorTree must contain exactly one root node");
    }
    root_node_ = parse_node(*first_node, requested_tree);
  }

  std::string mission_id_;
  std::unique_ptr<Node> root_node_;
  std::vector<std::string> declared_capabilities_;
  MissionRuntime runtime_{};
  bool terminal_{false};
};

MissionTree::MissionTree(const std::string & xml_path, std::string mission_id)
: impl_(std::make_unique<Impl>(xml_path, std::move(mission_id)))
{
  impl_->runtime_.mission_id = impl_->mission_id_;
}

MissionTree::~MissionTree() = default;
MissionTree::MissionTree(MissionTree &&) noexcept = default;
MissionTree & MissionTree::operator=(MissionTree &&) noexcept = default;

MissionTreeResult MissionTree::tick(
  const WorldState & world, const CapabilityState & capabilities,
  const std::vector<MissionEvent> & events, IActionExecutor & executor,
  const SafetySupervisor & safety)
{
  impl_->runtime_.airborne = world.airborne;
  MissionTreeResult result;
  result.runtime = impl_->runtime_;

  if (impl_->terminal_) {
    executor.update(world, world.now_s, world.dt_s);
    result.tree_status = impl_->runtime_.status == MissionStatus::succeeded ?
      NodeStatus::success : NodeStatus::failure;
    result.error = impl_->runtime_.failure_reason;
    return result;
  }

  if (impl_->runtime_.status == MissionStatus::idle) {
    impl_->runtime_.status = MissionStatus::running;
    impl_->runtime_.started_at_s = world.now_s;
  }
  executor.update(world, world.now_s, world.dt_s);
  auto safety_decision = safety.evaluate(world, capabilities, impl_->runtime_);
  result.safety = safety_decision;
  if (safety_decision.action == SafetyAction::pause) {
    if (!impl_->runtime_.active_action_id.empty()) {
      executor.pause(impl_->runtime_.active_action_id, world.now_s);
    }
    impl_->runtime_.status = MissionStatus::paused;
    result.tree_status = NodeStatus::running;
    result.runtime = impl_->runtime_;
    result.error = safety_decision.reason;
    return result;
  }
  if (safety_decision.action == SafetyAction::abort ||
    safety_decision.action == SafetyAction::emergency)
  {
    if (!impl_->runtime_.active_action_id.empty()) {
      executor.cancel(impl_->runtime_.active_action_id, world.now_s);
    }
    impl_->runtime_.status = MissionStatus::failed;
    impl_->runtime_.failure_reason = safety_decision.reason;
    impl_->runtime_.finished_at_s = world.now_s;
    impl_->terminal_ = true;
    result.tree_status = NodeStatus::failure;
    result.runtime = impl_->runtime_;
    result.error = safety_decision.reason;
    return result;
  }
  if (impl_->runtime_.status == MissionStatus::paused) {
    if (!impl_->runtime_.active_action_id.empty()) {
      executor.resume(impl_->runtime_.active_action_id, world.now_s);
    }
    impl_->runtime_.status = MissionStatus::running;
  }

  Impl::Context context{world, capabilities, events, executor, impl_->runtime_, {}, {}};
  const auto status = impl_->root_node_->tick(context);
  result.tree_status = status;
  result.emitted_events = context.emitted_events;
  result.error = context.error;
  if (status == NodeStatus::success) {
    impl_->runtime_.status = MissionStatus::succeeded;
    impl_->runtime_.finished_at_s = world.now_s;
    impl_->terminal_ = true;
  } else if (status == NodeStatus::failure) {
    impl_->runtime_.status = MissionStatus::failed;
    impl_->runtime_.finished_at_s = world.now_s;
    impl_->runtime_.failure_reason = context.error;
    impl_->terminal_ = true;
  }
  result.runtime = impl_->runtime_;
  return result;
}

void MissionTree::reset()
{
  impl_->root_node_->reset();
  impl_->runtime_ = {};
  impl_->runtime_.mission_id = impl_->mission_id_;
  impl_->terminal_ = false;
}

const MissionRuntime & MissionTree::runtime() const
{
  return impl_->runtime_;
}

const std::string & MissionTree::mission_id() const
{
  return impl_->mission_id_;
}

ValidationResult MissionTree::validate_capabilities(const CapabilityState & capabilities) const
{
  for (const auto & capability : impl_->declared_capabilities_) {
    if (!capabilities.supports(capability)) {
      return {false, "mission requires unsupported capability: " + capability};
    }
  }
  return {true, {}};
}

const char * to_string(const NodeStatus value)
{
  switch (value) {
    case NodeStatus::idle: return "idle";
    case NodeStatus::running: return "running";
    case NodeStatus::success: return "success";
    case NodeStatus::failure: return "failure";
  }
  return "idle";
}

}  // namespace mission_core
