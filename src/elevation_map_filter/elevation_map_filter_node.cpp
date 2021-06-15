/*
 *  Based on
 *  filters_demo_node.cpp
 *  by Peter Fankhauser (ETH Zurich, ANYbotics)
 * 
 *  Author: Matias Mattamala
 */


#include <elevation_map_filter/elevation_map_filter.hpp>

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "elevation_map_filter");
  ros::NodeHandle nodeHandle("~");

  bool success;
  ElevationMapFilter filter(nodeHandle, success);
  if (success)
    ros::spin();
  return 0;
}