#! /usr/bin/env python3

"""
ROS node for reading tf2 buffer and sending end-effector twist

Author: Ozan Tokatli <ozan.tokatli@gmail.com>
"""

import sys
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy


class ViveTeleoperation:
    def __init__(self, robot_name, twist_topic):
        self.robot_ee_twist_topic = '/' + robot_name + '/ee_goal_twist'
        self.twist_topic = twist_topic

        # Subscribe to controller twist topic
        self.controller_twist_sub = rospy.Subscriber(self.twist_topic,
                                                     TwistStamped,
                                                     self.controller_twist_cb,
                                                     queue_size=10)
        self.controller_twist = Twist()

        # Publisher for new controller twists
        self.controller_twist_pub = rospy.Publisher(self.robot_ee_twist_topic,
                                                    Twist,
                                                    queue_size=10)

    def controller_twist_cb(self, stamped_twist):
        self.controller_twist = stamped_twist.twist

    def send_ee_twist(self):
        self.controller_twist_pub.publish(self.controller_twist)


if __name__ == '__main__':
    rospy.init_node('vive_teleop_node')

    rospy.sleep(0.3)

    # Topics for left and right controller twists
    left_controller_twist_topic = '/left_controller_twist'
    right_controller_twist_topic = '/right_controller_twist'

    # Create robot instances
    robot_A = ViveTeleoperation('NNUF_A', left_controller_twist_topic)
    # robot_B = ViveTeleoperation('NNUF_B', right_controller_twist_topic)

    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():
        try:
            pass
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        robot_A.send_ee_twist()

        rate.sleep()
