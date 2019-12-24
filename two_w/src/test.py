#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def callback(msg):
    save = msg.pose.pose.position.x
    
rospy.init_node('check_odometry')
velocity_publisher = rospy.Publisher('/two_w/cmd_vel', Twist, queue_size=30)
odom_sub = rospy.Subscriber('/two_w/odom', Odometry, callback)

command = Twist()
command.linear.x = 0.5
velocity_publisher.publish(command)

rospy.spin()