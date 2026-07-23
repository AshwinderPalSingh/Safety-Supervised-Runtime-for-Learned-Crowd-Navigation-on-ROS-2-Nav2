#ifndef CROWD_NAV_PERCEPTION__GROUND_TRUTH_HUMAN_SOURCE_HPP_
#define CROWD_NAV_PERCEPTION__GROUND_TRUTH_HUMAN_SOURCE_HPP_

#include <deque>
#include <map>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "crowd_nav_pedestrians/msg/pedestrian_array.hpp"
#include "crowd_nav_perception/degradation_params.hpp"
#include "crowd_nav_perception/human_state_source.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crowd_nav_perception
{

// IMPLEMENTATION_PLAN.md S4.1/S4.2. Wraps Phase 4's ground-truth PedestrianArray topic,
// applies the degradation model, returns the degraded set. The message-adapter from
// crowd_nav_pedestrians::msg::PedestrianArray to HumanObservation lives entirely inside this
// class (onPedestrianArray) - callers of HumanStateSource never see the concrete message type,
// per S4.1's "this is the actual seam" framing.
//
// RNG substream separation (added v1.9, per review): this class's degradation RNG must never
// be seeded from the same raw value Phase 4's pedestrian_sim_node.py seeds its motion RNG from.
// They are already separate processes/languages today (Python pedestrian node, C++ perception
// library), so literal state-sharing isn't possible right now - but the *seed value* passed in
// here must still be a labeled substream, not the raw scenario seed reused, so that a future
// consolidation (or a harness that seeds both from "the same number" without thinking about it)
// can't silently reintroduce the coupling this requirement exists to prevent. Use
// deriveSubstreamSeed() below to get one; don't hand this class a raw scenario seed directly.
//
// Latency ring buffer (added v1.9, per review): keyed to sim timestamps carried on each
// buffered sample, not a tick count - getHumans() returns whichever buffered sample for a given
// human is the freshest one at least latency_s old, so the effective latency in seconds cannot
// silently drift if the upstream publish rate ever changes. tick_period_s in DegradationParams
// is used only to size how much history to retain (a bounded buffer, not for indexing), and is
// asserted plausible against observed inter-message timestamps rather than trusted blindly.
class GroundTruthHumanSource : public HumanStateSource
{
public:
  // Templated on the node type (added v1.15/Phase 8) so this works with both rclcpp::Node and
  // rclcpp_lifecycle::LifecycleNode - every nav2_core plugin receives the latter, not the
  // former, and both provide create_subscription with the same signature/return type via
  // different concrete classes. Found and fixed the hard way: Phase 7 first needed this inside
  // a live nav2_core::Controller and discovered the mismatch (docs/phase7-findings.md);
  // resolved here once Phase 8 actually needed live perception for its own live verification.
  // Defined inline (templates must be visible at the instantiation point).
  template<typename NodeT>
  GroundTruthHumanSource(
    const std::shared_ptr<NodeT> & node,
    const std::string & pedestrian_topic,
    const std::string & robot_pose_topic,
    const DegradationParams & params)
  : params_(params),
    rng_(params.degradation_seed),
    pos_noise_dist_(0.0, params.sigma_pos_m),
    vel_noise_dist_(0.0, params.sigma_vel_mps),
    dropout_dist_(params.dropout_prob)
  {
    pedestrian_sub_ = node->template create_subscription<crowd_nav_pedestrians::msg::PedestrianArray>(
      pedestrian_topic, 10,
      [this](const crowd_nav_pedestrians::msg::PedestrianArray::SharedPtr msg) {
        onPedestrianArray(msg);
      });
    robot_pose_sub_ = node->template create_subscription<geometry_msgs::msg::Pose>(
      robot_pose_topic, 10,
      [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
        onRobotPose(msg);
      });
  }

  // Test-only constructor: no ROS subscriptions at all, drive ingestPedestrian()/setRobotPose()
  // directly. Production code always uses the constructor above.
  explicit GroundTruthHumanSource(const DegradationParams & params);

  std::vector<HumanObservation> getHumans(const rclcpp::Time & query_time) override;

  // Derives an independent, deterministic 64-bit seed from one scenario seed plus a small
  // integer tag identifying the logical subsystem (e.g. 0 reserved for pedestrian motion - not
  // consumed here, documented for symmetry - 1 for this class's degradation noise/dropout).
  // Two calls with the same scenario_seed and different subsystem_tag produce unrelated
  // sequences; the same (scenario_seed, subsystem_tag) pair always reproduces the same seed.
  static uint64_t deriveSubstreamSeed(uint64_t scenario_seed, uint32_t subsystem_tag);

  // Pure ingestion logic, deliberately separated from the ROS message wrappers below (which
  // are thin field-extractors calling straight into these) so tests can drive this class
  // without standing up real publishers/subscriptions - the degradation/latency/RNG logic is
  // what needs coverage, not rclcpp's own message-passing.
  void ingestPedestrian(uint32_t id, double x, double y, double vx, double vy, const rclcpp::Time & stamp);
  void setRobotPose(double x, double y);

private:
  void onPedestrianArray(const crowd_nav_pedestrians::msg::PedestrianArray::SharedPtr msg);
  void onRobotPose(const geometry_msgs::msg::Pose::SharedPtr msg);

  std::optional<HumanObservation> degrade(const HumanObservation & raw);
  std::optional<HumanObservation> delayedLookup(uint32_t id, const rclcpp::Time & query_time);

  DegradationParams params_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> pos_noise_dist_;
  std::normal_distribution<double> vel_noise_dist_;
  std::bernoulli_distribution dropout_dist_;

  std::optional<std::pair<double, double>> robot_xy_;

  // Per-human history of raw (pre-latency, post-noise/dropout) observations, each tagged with
  // the sim timestamp it was received at - the actual latency mechanism, see class comment.
  std::map<uint32_t, std::deque<std::pair<rclcpp::Time, HumanObservation>>> history_;

  rclcpp::Subscription<crowd_nav_pedestrians::msg::PedestrianArray>::SharedPtr pedestrian_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot_pose_sub_;
};

}  // namespace crowd_nav_perception

#endif  // CROWD_NAV_PERCEPTION__GROUND_TRUTH_HUMAN_SOURCE_HPP_
