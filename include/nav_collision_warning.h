#ifndef nav_collision_warning_H
#define nav_collision_warning_H

#include "find_nearest_obstacle.h"
#include <geometry_msgs/PoseStamped.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

namespace robot_warnings
{

class nav_collision_warning
{
public:
  nav_collision_warning();

private:

  ros::NodeHandle n_;

  ros::Subscriber map_sub_;
  ros::Subscriber map_update_sub_;
  ros::Publisher coll_warning_pub_;
  ros::Publisher fwd_distance_pub_;

  nav_msgs::OccupancyGrid map_;

  void mapCB(nav_msgs::OccupancyGridConstPtr msg);
  void mapupdateCB(map_msgs::OccupancyGridUpdateConstPtr msg);
  void check_collisions();

  // Points on the robot chassis to check for collision
  geometry_msgs::PointStamped f_left_pt_;
  geometry_msgs::PointStamped f_right_pt_;
  std::vector<geometry_msgs::PointStamped> points_to_check_;

  // Get configurable max. speeds from ROS param server
  double scale_angular_;
  double scale_angular_turbo_;
  double scale_linear_;
  double scale_linear_turbo_;

  // These parameters are read from a yaml file
  double warning_threshold_;
  int num_pts_to_check_;
};

} // End robot_warnings namespace

#endif
