#include "crowd_nav_perception/lidar_clustering.hpp"

#include <cmath>
#include <utility>

namespace crowd_nav_perception
{

namespace
{
using Point = std::pair<double, double>;
using PointGroup = std::vector<Point>;

bool validRange(float r, double range_min, double range_max)
{
  return std::isfinite(r) && static_cast<double>(r) >= range_min &&
         static_cast<double>(r) <= range_max;
}

double dist(const Point & a, const Point & b)
{
  return std::hypot(a.first - b.first, a.second - b.second);
}

bool isFullCircleScan(size_t num_samples, double angle_increment)
{
  const double total_span = static_cast<double>(num_samples) * std::abs(angle_increment);
  return std::abs(total_span - 2.0 * M_PI) < std::abs(angle_increment) * 1.5;
}

ScanCluster toScanCluster(const PointGroup & group)
{
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const auto & p : group) {
    sum_x += p.first;
    sum_y += p.second;
  }
  ScanCluster c;
  c.num_points = static_cast<int>(group.size());
  c.x = sum_x / c.num_points;
  c.y = sum_y / c.num_points;
  c.width_m = dist(group.front(), group.back());
  return c;
}
}  // namespace

std::vector<ScanCluster> clusterScan(
  const std::vector<float> & ranges,
  double angle_min, double angle_increment,
  double range_min, double range_max,
  double break_distance_m)
{
  std::vector<PointGroup> groups;
  PointGroup current;
  Point prev{0.0, 0.0};
  bool have_prev = false;

  auto flush = [&]() {
      if (current.empty()) {return;}
      groups.push_back(std::move(current));
      current = PointGroup();
    };

  for (size_t i = 0; i < ranges.size(); ++i) {
    if (!validRange(ranges[i], range_min, range_max)) {
      flush();
      have_prev = false;
      continue;
    }
    const double angle = angle_min + static_cast<double>(i) * angle_increment;
    const Point p{
      static_cast<double>(ranges[i]) * std::cos(angle),
      static_cast<double>(ranges[i]) * std::sin(angle)};
    if (have_prev && dist(p, prev) > break_distance_m) {
      flush();
    }
    current.push_back(p);
    prev = p;
    have_prev = true;
  }
  flush();

  // Full-circle wraparound merge (see header comment): the array's last sample and first sample
  // are physically adjacent angles on a true 360-degree scan, even though nothing in the main
  // pass above ever compares them directly (it only ever compares each sample to the one
  // immediately before it in array order, which never wraps). Concatenating [last group's
  // points, first group's points] - in that order - means the merged group's own .front()/.back()
  // are exactly the two true outer extremes of the combined object (the standard width-proxy
  // this function already uses for every non-wrapped cluster), so no separate width formula is
  // needed for the merged case.
  if (groups.size() >= 2 && isFullCircleScan(ranges.size(), angle_increment) &&
    dist(groups.front().front(), groups.back().back()) <= break_distance_m)
  {
    PointGroup merged = std::move(groups.back());
    groups.pop_back();
    merged.insert(merged.end(), groups.front().begin(), groups.front().end());
    groups.front() = std::move(merged);
  }

  std::vector<ScanCluster> clusters;
  clusters.reserve(groups.size());
  for (const auto & g : groups) {
    clusters.push_back(toScanCluster(g));
  }
  return clusters;
}

}  // namespace crowd_nav_perception
