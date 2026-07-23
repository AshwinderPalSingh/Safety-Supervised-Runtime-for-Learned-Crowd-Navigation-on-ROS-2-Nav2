// Expected values below were computed independently in Python (not by re-invoking this
// project's own formula) - see the commit/phase6-findings.md for the exact script - so this
// test genuinely checks the C++ implementation against an external computation, not just that
// the C++ code agrees with itself.
#include "gtest/gtest.h"

#include "crowd_nav_policy_adapters/candidate_action_space.hpp"

using crowd_nav_policy_adapters::buildCandidateActionSpace;
using crowd_nav_policy_adapters::CandidateActionSpaceConfig;

namespace
{
CandidateActionSpaceConfig defaultConfig()
{
  CandidateActionSpaceConfig config;
  config.speed_samples = 5;
  config.rotation_samples = 16;
  config.time_step_s = 0.25;
  config.max_humans = 5;
  config.human_radius_m = 0.3;
  return config;
}
}  // namespace

TEST(CandidateActionSpaceConfig, CandidateCountAndFeatureDim)
{
  const auto config = defaultConfig();
  EXPECT_EQ(config.candidateCount(), 81u);
  EXPECT_EQ(config.featureDim(), 34u);  // 9 + 5*5
}

TEST(BuildCandidateActionSpace, IndexZeroIsStopAction)
{
  const auto actions = buildCandidateActionSpace(defaultConfig(), /*v_pref=*/1.0);
  ASSERT_EQ(actions.size(), 81u);
  EXPECT_DOUBLE_EQ(actions[0].vx, 0.0);
  EXPECT_DOUBLE_EQ(actions[0].vy, 0.0);
}

TEST(BuildCandidateActionSpace, MatchesIndependentlyComputedReferenceValues)
{
  const auto actions = buildCandidateActionSpace(defaultConfig(), /*v_pref=*/1.0);
  ASSERT_EQ(actions.size(), 81u);

  // rotation index 0 (rotation=0), speed index 0 -> candidate 1.
  EXPECT_NEAR(actions[1].vx, 0.12885124808584156, 1e-9);
  EXPECT_NEAR(actions[1].vy, 0.0, 1e-9);

  // rotation index 1, speed index 2 -> candidate 1 + 1*5 + 2 = 8.
  EXPECT_NEAR(actions[8].vx, 0.44203385055563177, 1e-9);
  EXPECT_NEAR(actions[8].vy, 0.18309641592814457, 1e-9);

  // rotation index 15 (last), speed index 4 (last, = v_pref) -> candidate 80 (last).
  EXPECT_NEAR(actions[80].vx, 0.9238795325112865, 1e-9);
  EXPECT_NEAR(actions[80].vy, -0.3826834323650904, 1e-9);
}

TEST(BuildCandidateActionSpace, ScalesLinearlyWithVPref)
{
  const auto actions_v1 = buildCandidateActionSpace(defaultConfig(), 1.0);
  const auto actions_v2 = buildCandidateActionSpace(defaultConfig(), 2.0);
  ASSERT_EQ(actions_v1.size(), actions_v2.size());
  for (size_t i = 0; i < actions_v1.size(); ++i) {
    EXPECT_NEAR(actions_v2[i].vx, actions_v1[i].vx * 2.0, 1e-9);
    EXPECT_NEAR(actions_v2[i].vy, actions_v1[i].vy * 2.0, 1e-9);
  }
}
