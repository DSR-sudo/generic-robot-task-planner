#pragma once

#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "mission_core/types.hpp"

namespace mission_core
{

struct ValidationResult
{
  bool accepted{false};
  std::string reason{};
};

using ActionValidator = std::function<ValidationResult(const ActionIntent &)>;

/// Registry used by platform adapters to validate action names and parameters at the boundary.
class CapabilityRegistry
{
public:
  void register_capability(std::string capability, ActionValidator validator = {});
  bool contains(const std::string & capability) const;
  ValidationResult validate(const ActionIntent & intent) const;
  std::vector<std::string> names() const;

private:
  std::map<std::string, ActionValidator> validators_{};
};

/// Platform-specific code owns this interface; mission_core never calls ROS, sockets or hardware.
class IActionExecutor
{
public:
  virtual ~IActionExecutor() = default;

  virtual CapabilityState capabilities(const WorldState & world) const = 0;
  virtual bool submit(const ActionIntent & intent, double now_s, std::string & reason) = 0;
  virtual void cancel(const std::string & action_id, double now_s) = 0;
  virtual void pause(const std::string & action_id, double now_s) = 0;
  virtual void resume(const std::string & action_id, double now_s) = 0;
  virtual void update(const WorldState & world, double now_s, double dt_s) = 0;
  virtual ActionFeedback feedback(const std::string & action_id) const = 0;
};

}  // namespace mission_core
