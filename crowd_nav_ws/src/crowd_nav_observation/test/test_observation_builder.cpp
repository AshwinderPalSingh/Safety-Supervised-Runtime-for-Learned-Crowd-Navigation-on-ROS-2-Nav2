#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "crowd_nav_observation/observation_builder.hpp"
#include "crowd_nav_observation/world_state.hpp"

namespace
{

using crowd_nav_observation::ObservationBuilder;
using crowd_nav_observation::WorldState;
using crowd_nav_perception::HumanObservation;

rclcpp::Time simTime(double seconds)
{
  return rclcpp::Time(
    static_cast<int32_t>(seconds),
    static_cast<uint32_t>((seconds - std::floor(seconds)) * 1e9),
    RCL_ROS_TIME);
}

// Test-only: CADRL.rotate() (crowd_nav/policy/cadrl.py, tkkim-robot/Gazebo-CrowdNav commit
// 9cad128d124f86bafe48d2cd11b5eee74bec77d9), transcribed exactly, kinematics == 'holonomic'
// branch only (matches the checkpoint, so theta_out is always 0).
// Deliberately NOT part of the public library: production rotation is Phase 8's SarlAdapter job.
// This exists only so the round-trip test can turn the builder's
// raw output into the same space the fixture's `rotated` column was computed in.
std::array<double, 13> referenceRotate(const std::array<double, 14> & s)
{
  // s: px,py,vx,vy,radius,gx,gy,v_pref,theta,px1,py1,vx1,vy1,radius1
  const double dx = s[5] - s[0];
  const double dy = s[6] - s[1];
  const double rot = std::atan2(dy, dx);
  const double cr = std::cos(rot);
  const double sr = std::sin(rot);

  const double dg = std::hypot(dx, dy);
  const double v_pref = s[7];
  const double vx = s[2] * cr + s[3] * sr;
  const double vy = s[3] * cr - s[2] * sr;
  const double radius = s[4];
  const double theta = 0.0;  // holonomic branch only
  const double vx1 = s[11] * cr + s[12] * sr;
  const double vy1 = s[12] * cr - s[11] * sr;
  const double px1 = (s[9] - s[0]) * cr + (s[10] - s[1]) * sr;
  const double py1 = (s[10] - s[1]) * cr - (s[9] - s[0]) * sr;
  const double radius1 = s[13];
  const double radius_sum = radius + radius1;
  const double da = std::hypot(s[0] - s[9], s[1] - s[10]);

  return {dg, v_pref, theta, radius, vx, vy, px1, py1, vx1, vy1, radius1, da, radius_sum};
}

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
    for (double & v : c.self_state) {
      iss >> v;
    }
    for (double & v : c.human_state) {
      iss >> v;
    }
    for (double & v : c.rotated) {
      iss >> v;
    }
    EXPECT_FALSE(iss.fail()) << "malformed fixture line: " << line;
    cases.push_back(c);
  }
  return cases;
}

WorldState worldStateFrom(
  const std::array<double, 9> & self_state, const std::array<double, 5> & human_state)
{
  WorldState state;
  state.robot.px = self_state[0];
  state.robot.py = self_state[1];
  state.robot.vx = self_state[2];
  state.robot.vy = self_state[3];
  state.robot.radius = self_state[4];
  state.robot.gx = self_state[5];
  state.robot.gy = self_state[6];
  state.robot.v_pref = self_state[7];
  state.robot.theta = self_state[8];

  HumanObservation h;
  h.id = 1;
  h.x = human_state[0];
  h.y = human_state[1];
  h.vx = human_state[2];
  h.vy = human_state[3];
  h.stamp = simTime(0.0);
  state.humans.push_back(h);
  return state;
}

}  // namespace

TEST(ObservationBuilder, HandComputedSingleHumanMatchesExpectedLayout)
{
  WorldState state;
  state.robot = {0.0, 0.0, 1.0, 0.0, 0.3, 5.0, 0.0, 1.0, 0.0};

  HumanObservation h;
  h.id = 7;
  h.x = 2.0;
  h.y = 1.0;
  h.vx = -0.3;
  h.vy = 0.1;
  h.stamp = simTime(0.0);
  state.humans.push_back(h);

  ObservationBuilder builder(/*max_humans=*/2, /*human_radius=*/0.3);
  const std::vector<double> out = builder.build(state);

  ASSERT_EQ(out.size(), 9u + 5u * 2u);
  // self state, verbatim
  EXPECT_DOUBLE_EQ(out[0], 0.0);
  EXPECT_DOUBLE_EQ(out[1], 0.0);
  EXPECT_DOUBLE_EQ(out[2], 1.0);
  EXPECT_DOUBLE_EQ(out[3], 0.0);
  EXPECT_DOUBLE_EQ(out[4], 0.3);
  EXPECT_DOUBLE_EQ(out[5], 5.0);
  EXPECT_DOUBLE_EQ(out[6], 0.0);
  EXPECT_DOUBLE_EQ(out[7], 1.0);
  EXPECT_DOUBLE_EQ(out[8], 0.0);
  // real human, slot 0
  EXPECT_DOUBLE_EQ(out[9], 2.0);
  EXPECT_DOUBLE_EQ(out[10], 1.0);
  EXPECT_DOUBLE_EQ(out[11], -0.3);
  EXPECT_DOUBLE_EQ(out[12], 0.1);
  EXPECT_DOUBLE_EQ(out[13], 0.3);  // config human_radius, not sensed
  // padding, slot 1: far away, stationary, zero radius
  EXPECT_DOUBLE_EQ(out[14], 0.0 + ObservationBuilder::kDummyDistanceM);
  EXPECT_DOUBLE_EQ(out[15], 0.0);
  EXPECT_DOUBLE_EQ(out[16], 0.0);
  EXPECT_DOUBLE_EQ(out[17], 0.0);
  EXPECT_DOUBLE_EQ(out[18], 0.0);
}

TEST(ObservationBuilder, OrdersHumansClosestFirstAndTruncates)
{
  WorldState state;
  state.robot = {0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 1.0, 0.0};

  HumanObservation far;
  far.id = 1;
  far.x = 10.0;
  far.y = 0.0;
  far.stamp = simTime(0.0);

  HumanObservation near;
  near.id = 2;
  near.x = 1.0;
  near.y = 0.0;
  near.stamp = simTime(0.0);

  HumanObservation mid;
  mid.id = 3;
  mid.x = 5.0;
  mid.y = 0.0;
  mid.stamp = simTime(0.0);

  state.humans = {far, near, mid};

  ObservationBuilder builder(/*max_humans=*/2, /*human_radius=*/0.25);
  const std::vector<double> out = builder.build(state);

  ASSERT_EQ(out.size(), 9u + 5u * 2u);
  EXPECT_DOUBLE_EQ(out[9], 1.0);   // near (closest) first
  EXPECT_DOUBLE_EQ(out[14], 5.0);  // mid second; far truncated away
}

// Round-trip against the reference implementation itself, not this project's understanding of
// it. Fixture generated by test/generate_reference_fixture.py
// calling the actual tkkim-robot/Gazebo-CrowdNav CADRL.rotate().
TEST(ObservationBuilder, RoundTripMatchesReferenceImplementation)
{
  const std::string fixture_path =
    std::string(CROWD_NAV_OBSERVATION_FIXTURE_DIR) + "/sarl_rotate_reference.txt";
  const std::vector<FixtureCase> cases = loadFixture(fixture_path);
  ASSERT_GE(cases.size(), 1u) << "fixture produced no cases - check generation script ran";

  for (size_t i = 0; i < cases.size(); ++i) {
    SCOPED_TRACE("case " + std::to_string(i));
    const auto & c = cases[i];

    // human_state[4] (radius1) varies per synthetic fixture case; ObservationBuilder treats
    // radius as a config constant (see class comment), so configure it per-case here to match
    // what the fixture's Python-side rotation assumed. Position/velocity rotation - the part
    // with real trigonometry to get wrong - is still exercised end-to-end unmodified.
    ObservationBuilder builder(/*max_humans=*/1, /*human_radius=*/c.human_state[4]);

    WorldState state = worldStateFrom(c.self_state, c.human_state);
    const std::vector<double> raw = builder.build(state);
    ASSERT_EQ(raw.size(), 14u);

    std::array<double, 14> raw_arr;
    std::copy(raw.begin(), raw.end(), raw_arr.begin());
    const std::array<double, 13> rotated = referenceRotate(raw_arr);

    for (size_t k = 0; k < 13; ++k) {
      EXPECT_NEAR(rotated[k], c.rotated[k], 1e-6) << "column index " << k;
    }
  }
}
