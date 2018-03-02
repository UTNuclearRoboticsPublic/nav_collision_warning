
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


robot_warnings::nav_collision_warning::nav_collision_warning() :
  tf_l_(ros::Duration(10))
{
  // Read params from yaml file
  ROS_INFO_STREAM("----------------------------------");
  ROS_INFO_STREAM("--NAV COLLISION WARNING SETTINGS--");
  n_.getParam("/nav_collision_warning/num_pts_to_check", num_pts_to_check_);
  ROS_INFO_STREAM("Number of points around robot to check: " << num_pts_to_check_);
  std::string costmap_topic;
  n_.getParam("/nav_collision_warning/costmap_topic", costmap_topic);
  ROS_INFO_STREAM("Costmap topic: " << costmap_topic);
  std::string costmap_updates_topic;
  n_.getParam("/nav_collision_warning/costmap_updates_topic", costmap_updates_topic);
  ROS_INFO_STREAM("Costmap_updates topic: " << costmap_updates_topic);
  n_.getParam("/nav_collision_warning/collision_ellipse_x", collision_ellipse_x_);
  ROS_INFO_STREAM("X-dimension of collision ellipse: " << collision_ellipse_x_);
  n_.getParam("/nav_collision_warning/collision_ellipse_y", collision_ellipse_y_);
  ROS_INFO_STREAM("Y-dimension of collision ellipse: " << collision_ellipse_y_);
  ROS_INFO_STREAM("----------------------------------");

  // Need to subscribe to costmap and costmap_updates
  // Costmap is only updated when the robot is translating
  map_sub_ = n_.subscribe(costmap_topic, 1, &nav_collision_warning::mapCB, this);
  map_update_sub_ = n_.subscribe(costmap_updates_topic, 1, &nav_collision_warning::mapupdateCB, this);

  spd_fraction_pub_ = n_.advertise<std_msgs::Float64>("nav_collision_warning/spd_fraction", 1);

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
    n_.getParam("/nav_collision_warning/enable_nav_collision", checking_enabled);

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
    double x_to_obs = 0.;
    double y_to_obs = 0.;
    double min_dist = 1000.;
    robot_warnings::results results;
    for (int i=0; i<points_to_check_.size(); i++)
    {
      results = find_nearest_obstacle(points_to_check_[i], map_, tf_l_);
      if (results.min_dist < min_dist)
      {
        x_to_obs = results.x_to_obstacle;
        y_to_obs = results.y_to_obstacle;
      }
    }

	// Publish speed fraction.
  // This is based on the equation of an ellipse.
	std_msgs::Float64 spd_frac;
	spd_frac.data = x_to_obs*x_to_obs/(collision_ellipse_x_*collision_ellipse_x_) +
    y_to_obs*y_to_obs/(collision_ellipse_y_*collision_ellipse_y_);

	// If very far away, don't slow down
	if ( spd_frac.data >= 1. )
	  spd_frac.data = 1.;

	spd_fraction_pub_.publish(spd_frac);
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
