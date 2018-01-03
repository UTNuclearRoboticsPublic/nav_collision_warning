#include <find_nearest_obstacle.h>

// Find the nearest obstacle to a single point, and the x-distance to it in "base_link" frame.

// information of the map
struct map_inf {
    double size_x;
    double size_y;
    double scale;
    double origin_x;
    double origin_y;
};


robot_warnings::results robot_warnings::find_nearest_obstacle(geometry_msgs::PointStamped & pt,
                                    nav_msgs::OccupancyGrid & map){

    // remapping information in info variable
    map_inf map_info;
    map_info.size_x = map.info.width;
    map_info.size_y = map.info.height;
    map_info.scale = map.info.resolution;
    map_info.origin_x = map.info.origin.position.x + (map_info.size_x / 2) * map_info.scale;
    map_info.origin_y = map.info.origin.position.y + (map_info.size_y / 2) * map_info.scale;

    // map (world reference)
    double gt_x = map.info.origin.position.x;
    double gt_y = map.info.origin.position.y;

    tf::TransformListener tf_l(ros::Duration(10));
    geometry_msgs::PointStamped new_pt;
    bool success = false;

    // next line to avoid tf extrapolation into the past (not good code)
    map.header.stamp = pt.header.stamp = ros::Time(0);

    // transform in case the pose to evaluate and the point are not in the same
    // reference frame
    try{
        tf_l.waitForTransform(map.header.frame_id,
                              pt.header.frame_id, ros::Time(0), ros::Duration(1));
        tf_l.transformPoint(map.header.frame_id, pt, new_pt);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    // robot in map frame reference
    double robot_x = new_pt.point.x;
    double robot_y = new_pt.point.y;
    
    // variables to keep the distance
    double dist_sq, min_dist = 100000;

    double w_x, w_y;  // world cordinates
    double obstacle_x, obstacle_y;  // saved world cordinates
    // more efficient first with heigh and then width because map information
    // is in row-major order
    for (std::size_t j=0; j < map_info.size_y; j++) {
        for (std::size_t i=0; i < map_info.size_x; i++) {
            if(map.data[MAP_INDEX(map_info, i, j)]==100){
                // convert to world position
                w_x = MAP_WXGX(map_info, i);
                w_y = MAP_WYGY(map_info, j);
                dist_sq = pow(w_x-robot_x,2)+pow(w_y-robot_y,2);
                if (dist_sq<min_dist){
                    min_dist = dist_sq;
                    obstacle_x = w_x;
                    obstacle_y = w_y;
                }
            }
        }
    }
    min_dist = sqrt(min_dist);

    // debug information
    //ROS_WARN_STREAM("Min. Distance: "<< min_dist);
    //ROS_WARN_STREAM("Robot is at position "<< robot_x <<","<< robot_y);
    //ROS_WARN_STREAM("Obstacle is at position "<< obstacle_x <<","<< obstacle_y);

    // We want output in the base_link frame.
    // Rotate this vector (which is currently in the world frame)
    geometry_msgs::Vector3Stamped output_vec;
    output_vec.vector.x = obstacle_x-robot_x;
    output_vec.vector.y = obstacle_y-robot_y;
    double odom_magnitude = pow( pow(output_vec.vector.x,2)+pow(output_vec.vector.y,2), 0.5);
    output_vec.vector.z = 0.;
    output_vec.header.frame_id = map.header.frame_id;

    try{
        tf_l.waitForTransform(output_vec.header.frame_id,
                              "base_link", ros::Time(0), ros::Duration(1));
        tf_l.transformVector("base_link", output_vec, output_vec);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    //ROS_INFO_STREAM(output_vec);

    results results;
    results.x_to_obstacle = output_vec.vector.x;
    results.min_dist = min_dist;

    return results;
}
