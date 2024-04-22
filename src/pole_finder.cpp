#include "ymsm_pole_finder/pole_finder_node.h"

int main(int argc, char **argv)
{
  //ノードの初期化
  ros::init(argc, argv, "pole_finder");
  ymsm_pole_finder::PoleFinderNode node;
  ros::spin();
  return 0;
}