# nav_collision_warning

Check the distance to obstacles as a robot base moves.

Usage:

Adjust the settings in config/settings.yaml. You can check the distance to as many points around the robot chassis as you need. However, this is a computationally-intensive program, so I would suggest fewer.


Then,

roslaunch nav_collision_warning nav_collision_warning.launch

You can pause the collision checking at any time by setting this parameter to false:

rosparam set /nav_collision_warning/enable_nav_collision false


Resume again with:

rosparam set /nav_collision_warning/enable_nav_collision true


When an obstacle is close, the node will publish true on /nav_collision_warning/imminent_collision. The node will always publish the forward distance to the nearest obstacle on /nav_collision_warning/fwd_distance_to_obst. (Presuming your robot can't instantly jump sideways, the forward direction is more important.) You could use this fwd_distance to slow down proportionally as an obstacle is approached.
