#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <iterator>
#include <vector>

// some definitions of functions
#define MAP_INDEX(map, i, j) ((i) + (j) * map.size_x)
#define MAP_WXGX(map, i) (map.origin_x + (i - map.size_x / 2) * map.scale)
#define MAP_WYGY(map, j) (map.origin_y + (j - map.size_y / 2) * map.scale)

namespace robot_warnings
{
  struct results
  {
    double min_dist;
    double x_to_obstacle;
  };

 results find_nearest_obstacle(geometry_msgs::PointStamped & pt,
                                    nav_msgs::OccupancyGrid & map);
}
