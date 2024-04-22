#include "ymsm_pole_finder/pole_finder_node.h"

#include <limits>
#include <numeric>

namespace ymsm_pole_finder
{

PoleFinderNode::PoleFinderNode() :
  ros::NodeHandle(),
  scan_subscriber_(this->subscribe("scan", 1, &PoleFinderNode::scanToPoles, this)),
  poles_publisher_(this->advertise<geometry_msgs::PoseArray>("poles", 1, false))
{
}

void PoleFinderNode::scanToPoles(
  const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  geometry_msgs::PoseArray poles_msg;
  poles_msg.header = scan_msg->header;

  std::vector<float> pole_ranges;

  float range_angle = scan_msg->angle_min;
  float range_last = std::numeric_limits<float>::quiet_NaN();

  auto write_pole_ranges = [&]()
  {
    geometry_msgs::Pose pole;
    const auto pole_range = std::accumulate(
      pole_ranges.begin(), pole_ranges.end(), 0) / pole_ranges.size();
    const auto pole_angle = range_angle
      - (pole_ranges.size() + 1) * scan_msg->angle_increment * 0.5;
    pole.position.x = pole_range * std::cos(pole_angle);
    pole.position.y = pole_range * std::sin(pole_angle);
    pole.position.z = 0;
    pole.orientation.x = 0;
    pole.orientation.y = 0;
    pole.orientation.z = 0;
    pole.orientation.w = 1;
    poles_msg.poses.push_back(pole);
    pole_ranges.clear();
  };

  for (const auto &range : scan_msg->ranges)
  {
    const auto range_diff = range - range_last;
    
    if (std::isnormal(range_last) 
      && (!std::isnormal(range_last) || range_diff * range_diff > 0.1 * 0.1))
    {
      write_pole_ranges();
    }

    if (std::isnormal(range))
    {
      pole_ranges.push_back(range);
    }
  
    range_last = range;
    range_angle += scan_msg->angle_increment;
  }

  if (!pole_ranges.empty())
  {
    write_pole_ranges();
  }

  poles_publisher_.publish(poles_msg);

  ++seq_;
}

}