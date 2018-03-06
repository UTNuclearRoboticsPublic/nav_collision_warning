# nav_collision_warning

Check the distance to obstacles as a robot base moves.

Usage:

Adjust the settings in config/settings.yaml. You can check the distance to as many points around the robot chassis as you need. However, this is a computationally-intensive program, so I would suggest fewer.


Then,

**roslaunch nav_collision_warning nav_collision_warning.launch**

You can pause the collision checking at any time by setting this parameter to false:

**rosparam set /nav_collision_warning/enable_nav_collision false**


Resume again with:

**rosparam set /nav_collision_warning/enable_nav_collision true**


The node publishes **/nav_collision_warning/spd_fraction**. If an obstacle is close, this drops below 1.0 and can be used to scale the velocity of the robot.

See a plot of how spd_fraction varies by running...

**roslaunch nav_collision_warning plot_collision_ellipse.launch**

Change its dimensions in the config file.
