#!/usr/bin/env python

"""Plot the ellipse that converts an obstacle to spd_fraction.

Andy Zelenak
"""

import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import rospy

if __name__ == "__main__":
	rospy.init_node("plot_spd_fraction")

	mpl.rcParams['legend.fontsize'] = 10

	fig = plt.figure()
	ax = fig.gca(projection='3d')

	steps = 50

	# collision ellipse x-length, y-length
	a = rospy.get_param("/nav_collision_warning/collision_ellipse_x")
	b = rospy.get_param("/nav_collision_warning/collision_ellipse_y")

	x_range = np.linspace(-3, 3, steps)
	y_range = np.linspace(-3, 3, steps)

	for x in np.nditer(x_range):
		for y in np.nditer(y_range):
			if ( x**2/a**2 + y**2/b**2 ) < 1 :
				spd_fraction = x**2/a**2 + y**2/b**2
			else:
				spd_fraction = 1
			ax.scatter(x, y, spd_fraction)

	ax.set_xlabel('x in base_link [m]')
	ax.set_ylabel('y in base_link [m]')
	ax.set_zlabel('spd_fraction')

	plt.show()
