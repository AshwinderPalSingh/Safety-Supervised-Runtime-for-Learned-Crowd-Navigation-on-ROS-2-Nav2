#include "crowd_nav_policy_adapters/candidate_action_space.hpp"

#include <cmath>
#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace crowd_nav_policy_adapters
{

CandidateActionSpaceConfig loadCandidateActionSpaceConfig(const std::string & yaml_path)
{
  YAML::Node node = YAML::LoadFile(yaml_path);

  CandidateActionSpaceConfig config;
  config.speed_samples = node["speed_samples"].as<int>();
  config.rotation_samples = node["rotation_samples"].as<int>();
  config.time_step_s = node["time_step_s"].as<double>();
  config.max_humans = node["max_humans"].as<size_t>();
  config.human_radius_m = node["human_radius_m"].as<double>();
  config.policy_radius_m = node["policy_radius_m"].as<double>();
  config.policy_v_pref_mps = node["policy_v_pref_mps"].as<double>();

  if (config.speed_samples <= 0 || config.rotation_samples <= 0) {
    throw std::runtime_error(
      "loadCandidateActionSpaceConfig: speed_samples/rotation_samples must be positive: " +
      yaml_path);
  }
  return config;
}

std::vector<CandidateAction> buildCandidateActionSpace(
  const CandidateActionSpaceConfig & config, double v_pref)
{
  std::vector<CandidateAction> actions;
  actions.reserve(config.candidateCount());
  actions.push_back({0.0, 0.0});  // stop action, always index 0

  std::vector<double> speeds(config.speed_samples);
  for (int i = 0; i < config.speed_samples; ++i) {
    speeds[i] =
      (std::exp(static_cast<double>(i + 1) / config.speed_samples) - 1.0) /
      (M_E - 1.0) * v_pref;
  }

  std::vector<double> rotations(config.rotation_samples);
  for (int j = 0; j < config.rotation_samples; ++j) {
    rotations[j] = j * (2.0 * M_PI / config.rotation_samples);
  }

  // rotation outer, speed inner - matches itertools.product(rotations, speeds)'s order.
  for (double rotation : rotations) {
    for (double speed : speeds) {
      actions.push_back({speed * std::cos(rotation), speed * std::sin(rotation)});
    }
  }
  return actions;
}

}  // namespace crowd_nav_policy_adapters
