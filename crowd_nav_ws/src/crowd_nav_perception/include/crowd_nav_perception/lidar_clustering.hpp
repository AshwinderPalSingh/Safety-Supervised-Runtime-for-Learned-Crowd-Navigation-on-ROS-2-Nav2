#ifndef CROWD_NAV_PERCEPTION__LIDAR_CLUSTERING_HPP_
#define CROWD_NAV_PERCEPTION__LIDAR_CLUSTERING_HPP_

#include <vector>

namespace crowd_nav_perception
{

// One contiguous group of LiDAR returns that survived range validity + adaptive-breakpoint
// segmentation, in the sensor's own 2D frame (not yet transformed to map frame - that happens
// one layer up, in LidarHumanTrackerSource, via the robot's real TF tree).
struct ScanCluster
{
  double x = 0.0;      // centroid, sensor frame
  double y = 0.0;
  double width_m = 0.0;  // straight-line distance between the cluster's first and last point -
                          // a cheap, standard proxy for object width, used one layer up to
                          // reject clusters too wide/narrow to plausibly be a human leg/torso
                          // cross-section (a wall segment or shelf pole clusters too, but at a
                          // width this check can usually rule out).
  int num_points = 0;
};

// Adaptive-breakpoint segmentation (a standard, simple 2D-LiDAR clustering method - Borges &
// Aguiar 2004 is the usual citation): walk consecutive valid returns in angular order, start a
// new cluster whenever the Cartesian distance between consecutive points exceeds
// break_distance_m. Deliberately not a fancier method (RANSAC circle fits, learned leg
// detectors like DR-SPAAM/DROW): this robot's budget LiDAR spec (README's Known Limitations -
// ~180 deg / 8 m / ~1 deg resolution) doesn't have the angular resolution to make those
// meaningfully more accurate, and a classical, inspectable method is the honest choice for a
// first real (non-ground-truth) perception pipeline, not a placeholder for a better one.
//
// ranges.size() is assumed to be the number of samples between angle_min and
// angle_min + (n-1)*angle_increment, matching sensor_msgs/LaserScan's own layout. A range
// outside [range_min, range_max] or non-finite is treated as no-return and cannot start or
// extend a cluster, the same convention LaserScan itself uses for "nothing seen at this angle."
//
// Full-circle wraparound: if the scan's total angular span is within one sample of a full 2*pi
// (a genuine 360-degree LiDAR, not this project's own sim sensor - masked to ~180 degrees at
// the source for an unrelated gz-sim rendering bug, README's Known Limitations), the last
// sample's angle is physically adjacent to the first sample's, even though they sit at opposite
// ends of the ranges array. Without handling this, a human standing anywhere near the scan's
// zero-angle seam gets cut into two separate half-width clusters, each likely to fail the
// human-width filter one layer up - a real gap on a true 360-degree sensor (the one this
// project's own physical LiDAR is, per its spec) even though the current 180-degree-masked sim
// sensor can never expose it, since its own "seam" always sits in the always-empty region behind
// the robot. Handled by merging the first and last clusters after the main pass, when both exist
// and their endpoints are within break_distance_m of each other.
std::vector<ScanCluster> clusterScan(
  const std::vector<float> & ranges,
  double angle_min, double angle_increment,
  double range_min, double range_max,
  double break_distance_m = 0.15);

}  // namespace crowd_nav_perception

#endif  // CROWD_NAV_PERCEPTION__LIDAR_CLUSTERING_HPP_
