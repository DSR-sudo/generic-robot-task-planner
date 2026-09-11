#include "mission_core/executor.hpp"

#include <stdexcept>

namespace mission_core
{

void CapabilityRegistry::register_capability(std::string capability, ActionValidator validator)
{
  if (capability.empty()) {
    throw std::invalid_argument("capability name must not be empty");
  }
  if (validators_.find(capability) != validators_.end()) {
    throw std::invalid_argument("capability already registered: " + capability);
  }
  validators_.emplace(std::move(capability), std::move(validator));
}

bool CapabilityRegistry::contains(const std::string & capability) const
{
  return validators_.find(capability) != validators_.end();
}

ValidationResult CapabilityRegistry::validate(const ActionIntent & intent) const
{
  if (intent.action_id.empty()) {
    return {false, "action_id must not be empty"};
  }
  if (intent.capability.empty()) {
    return {false, "capability must not be empty"};
  }
  const auto found = validators_.find(intent.capability);
  if (found == validators_.end()) {
    return {false, "unsupported capability: " + intent.capability};
  }
  if (!found->second) {
    return {true, {}};
  }
  return found->second(intent);
}

std::vector<std::string> CapabilityRegistry::names() const
{
  std::vector<std::string> result;
  result.reserve(validators_.size());
  for (const auto & entry : validators_) {
    result.push_back(entry.first);
  }
  return result;
}

}  // namespace mission_core
