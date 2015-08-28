#include <ros/ros.h>
#include "sim_reset/sim_reset.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_reset_node");
  ros::NodeHandle nh;

  sim_reset::simReset Thing;

  ros::spin();

  return 0;
}
