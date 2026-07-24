// Verifies the PRODUCTION rotate() (used by SarlAdapter, Phase 8) against the same checked-in
// reference fixture Phase 5's round-trip test used - generated from the real
// tkkim-robot/Gazebo-CrowdNav CADRL.rotate(), not a transcription (IMPLEMENTATION_PLAN.md
// S4.1.2/S4.7).
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "crowd_nav_policy_adapters/sarl_rotate.hpp"

using crowd_nav_observation::RobotSelfState;
using crowd_nav_policy_adapters::rotate;

namespace
{
struct FixtureCase
{
  std::array<double, 9> self_state;
  std::array<double, 5> human_state;
  std::array<double, 13> rotated;
};

std::vector<FixtureCase> loadFixture(const std::string & path)
{
  std::ifstream in(path);
  EXPECT_TRUE(in.is_open()) << "could not open fixture: " << path;
  std::vector<FixtureCase> cases;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    FixtureCase c;
    for (double & v : c.self_state) {iss >> v;}
    for (double & v : c.human_state) {iss >> v;}
    for (double & v : c.rotated) {iss >> v;}
    EXPECT_FALSE(iss.fail()) << "malformed fixture line: " << line;
    cases.push_back(c);
  }
  return cases;
}
}  // namespace

TEST(SarlRotate, MatchesReferenceImplementationFixture)
{
  const std::string fixture_path =
    std::string(CROWD_NAV_POLICY_ADAPTERS_PACKAGE_DIR) + "/test/fixtures/sarl_rotate_reference.txt";
  const auto cases = loadFixture(fixture_path);
  ASSERT_GE(cases.size(), 1u);

  for (size_t i = 0; i < cases.size(); ++i) {
    SCOPED_TRACE("case " + std::to_string(i));
    const auto & c = cases[i];

    RobotSelfState self;
    self.px = c.self_state[0];
    self.py = c.self_state[1];
    self.vx = c.self_state[2];
    self.vy = c.self_state[3];
    self.radius = c.self_state[4];
    self.gx = c.self_state[5];
    self.gy = c.self_state[6];
    self.v_pref = c.self_state[7];
    self.theta = c.self_state[8];

    const auto result = rotate(
      self, c.human_state[0], c.human_state[1], c.human_state[2], c.human_state[3],
      c.human_state[4]);

    for (size_t k = 0; k < 13; ++k) {
      EXPECT_NEAR(result[k], c.rotated[k], 1e-6) << "column index " << k;
    }
  }
}
