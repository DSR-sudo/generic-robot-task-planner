#pragma once

#include <memory>
#include <string>
#include <vector>

#include "mission_core/executor.hpp"
#include "mission_core/safety.hpp"

namespace mission_core
{

enum class NodeStatus
{
  idle,
  running,
  success,
  failure
};

struct MissionTreeResult
{
  NodeStatus tree_status{NodeStatus::idle};
  MissionRuntime runtime{};
  SafetyDecision safety{};
  std::vector<MissionEvent> emitted_events{};
  std::string error{};
};

/// A dependency-free XML mission runner with a small, explicit node vocabulary.
class MissionTree
{
public:
  MissionTree(const std::string & xml_path, std::string mission_id = "mission");
  ~MissionTree();
  MissionTree(MissionTree &&) noexcept;
  MissionTree & operator=(MissionTree &&) noexcept;
  MissionTree(const MissionTree &) = delete;
  MissionTree & operator=(const MissionTree &) = delete;

  MissionTreeResult tick(
    const WorldState & world, const CapabilityState & capabilities,
    const std::vector<MissionEvent> & events, IActionExecutor & executor,
    const SafetySupervisor & safety);
  /// Reject only capabilities that are not registered by the selected platform.
  /// Temporarily unavailable capabilities remain valid and are handled by RequireCapability.
  ValidationResult validate_capabilities(const CapabilityState & capabilities) const;
  void reset();

  const MissionRuntime & runtime() const;
  const std::string & mission_id() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

const char * to_string(NodeStatus value);

}  // namespace mission_core
