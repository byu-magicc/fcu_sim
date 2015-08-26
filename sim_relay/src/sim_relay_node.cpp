#include <ros/ros.h>
#include "sim_relay/sim_relay.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_relay_node");
  ros::NodeHandle nh;

  sim_relay::simRelay Thing;

  ros::spin();

  return 0;
}
