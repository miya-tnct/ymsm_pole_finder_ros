#include "ymsm_pole_finder/pole_finder_node.h"

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
  poles_publisher_.publish(poles_msg);
}

}