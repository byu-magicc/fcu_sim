#include <ros/ros.h>
#include "attitude_controller/attitude_controller.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "attitude_controller_node");
  ros::NodeHandle nh;

  attitude_controller::attitudeController Thing;

  ros::spin();

  return 0;
}
