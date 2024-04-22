#ifndef YMSM_POLE_FINDER_POLE_FINDER_H_
#define YMSM_POLE_FINDER_POLE_FINDER_H_

#include <cstdint>

#include "geometry_msgs/PoseArray.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

namespace ymsm_pole_finder
{

class PoleFinderNode : public ros::NodeHandle
{
public:
  PoleFinderNode();

private:
  void scanToPoles(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  std::uint32_t seq_;

  ros::Publisher poles_publisher_;
  ros::Subscriber scan_subscriber_;
};

}

#endif