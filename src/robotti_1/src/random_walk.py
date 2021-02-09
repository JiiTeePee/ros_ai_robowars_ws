#! usr/bin/python3

import rospy, random, math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.init_node("random_walk_robot1")

        