
///////////////////////////////////////////////////////////////////////////////
//      Title     : nav_collision_warning.cpp
//      Project   : vaultbot_nrg
//      Created   : 10/19/2017
//      Author    : Andy Zelenak
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All rights reserved.
//                 
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include "nav_collision_warning.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_collision_warning");

  robot_warnings::nav_collision_warning warn;

  return 0;
}


robot_warnings::nav_collision_warning::nav_collision_warning()
{
  // Need to subscribe to costmap and costmap_updates
  // Costmap is only updated when the robot is translating
  map_sub_ = n_.subscribe("/move_base/local_costmap/costmap", 1, &nav_collision_warning::mapCB, this);
  map_update_sub_ = n_.subscribe("/move_base/local_costmap/costmap_updates", 1, &nav_collision_warning::mapupdateCB, this);

  coll_warning_pub_ = n_.advertise<std_msgs::Bool>("nav_collision_warning/imminent_collision", 1);
  fwd_distance_pub_ = n_.advertise<std_msgs::Float64>("nav_collision_warning/fwd_distance_to_obst", 1);

  // Read params from yaml file
  ROS_INFO_STREAM("----------------------------------");
  ROS_INFO_STREAM("--NAV COLLISION WARNING SETTINGS--");
  n_.getParam("/nav_collision_warning/warning_threshold", warning_threshold_);
  ROS_INFO_STREAM("Threshold distance for publishing a collision warning [m]: " << warning_threshold_);
  n_.getParam("/nav_collision_warning/num_pts_to_check", num_pts_to_check_);
  ROS_INFO_STREAM("Number of points around robot to check: " << num_pts_to_check_);
  ROS_INFO_STREAM("----------------------------------");

  // Initialize points to be checked
  double x, y;
  geometry_msgs::PointStamped pt;
  pt.header.frame_id = "base_link";
  pt.point.z = 0.;
  for (int i=0; i<num_pts_to_check_; i++)
  {
    n_.getParam("/nav_collision_warning/pts_to_check/pt"+std::to_string(i)+"/x", x);
    n_.getParam("/nav_collision_warning/pts_to_check/pt"+std::to_string(i)+"/y", y);
    pt.point.x = x;
    pt.point.y = y;
    points_to_check_.push_back(pt);
  }

  bool checking_enabled = true;
  while ( ros::ok() )
  {
    n_.getParam("/enable_nav_collision", checking_enabled);

    if (checking_enabled)
    {
      // update the map
      ros::spinOnce();

      // Check for collisions
      check_collisions();
    }

    ros::Duration(0.05).sleep();
  }
}

void robot_warnings::nav_collision_warning::check_collisions()
{
  // if a map has been received
  if (map_.header.frame_id != "" )
  {

    // Check these pts around the chassis
    double min_dist = 100000.;
    double x_to_obstacle = 0.;
    robot_warnings::results results;
    for (int i=0; i<points_to_check_.size(); i++)
    {
      results = find_nearest_obstacle(points_to_check_[i], map_);
      if (results.min_dist < min_dist)
      {
        min_dist = results.min_dist;
        x_to_obstacle = results.x_to_obstacle;
      }
    }

    // Publish forward-distance (x in base_link) to nearest obstacle
    std_msgs::Float64 fwd_distance_msg;
    fwd_distance_msg.data = x_to_obstacle;
    fwd_distance_pub_.publish(fwd_distance_msg);

    // Publish a warning if close to obstacle
    if ( min_dist < 0.5 )
    {
      std_msgs::Bool warning_msg;
      warning_msg.data = true;
      coll_warning_pub_.publish(warning_msg);
    }
  }

  return;
}


void robot_warnings::nav_collision_warning::mapCB(nav_msgs::OccupancyGridConstPtr msg)
{
  map_ = *msg;
}

// Update the map member.
// This assumes the update is the same size as the map msg.
void robot_warnings::nav_collision_warning::mapupdateCB(map_msgs::OccupancyGridUpdateConstPtr msg)
{
  map_.data = msg->data;
}
