
from __future__ import print_function
import os
import re
import random
from math import *
import tf
import tf2_ros
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# Setup prothonics.
#blindRobot = prothonics.Prothonics(100, 100)
#blindRobot.useBrain().useLearning().learnKnoledgeBaseFromFile("behaviour.pl")
#sub = rospy.Subscriber('scan', LaserScan, callback=blindRobot.useSense)

class Robot():
    def __init__(self):
        self.radians = [pi/2, pi, -pi/2]
        rospy.init_node('robot', anonymous=True)   
        rospy.on_shutdown(self.shutdown)
        self.timer = rospy.Rate(20)
        self.odometry = Odometry()
        # self.laserscan = LaserScan()
        self.velocity_publisher = rospy.Publisher(
                                                    "/cmd_vel", 
                                                     Twist, 
                                                     queue_size=10
                                                )
        self.timer.sleep()
        rospy.Subscriber(
                                                    "/odom", 
                                                       Odometry, 
                                                       self.odom_callback   
                                                )
        self.timer.sleep()
        # rospy.Subscriber(
        #                                             "/scan",
        #                                              LaserScan, 
        #                                              self.__bumper_callback
        #                                         )
        # self.timer.sleep()
        self.rotate_by(random.choice(self.radians))

    def odom_callback(self, odometry):
        self.odometry = odometry
        rospy.loginfo(
                        "Coordinate:: X: %f, Y: %f"
                        %(
                            self.odometry.pose.pose.position.x,
                            self.odometry.pose.pose.position.y
                         )
                     )
    
    def shutdown(self):
        self.velocity_publisher.publish(Twist())
        self.timer.sleep()
    
    def rotate_by(self, radians):
        if (radians>0):
            angular_velocity_z = -0.3
        else:
            angular_velocity_z = 0.3
        while radians > 2*pi :
            radians -= 2*pi
        while radians < 0:
            radians += 2*pi
        rospy.loginfo("rotate_by function called, rotating by: %f" %radians)
        previous_yaw = tf.transformations.euler_from_quaternion(
                                                                    [
                                                                        self.odometry.pose.pose.orientation.x,
                                                                        self.odometry.pose.pose.orientation.y,
                                                                        self.odometry.pose.pose.orientation.z,
                                                                        self.odometry.pose.pose.orientation.w
                                                                    ]
                                                               )[2]
        self.timer.sleep()
        angle_turned = 0.0
        while True:
            self.velocity_publisher.publish(
                                                Vector3(0, 0, 0),
                                                Vector3(0, 0, angular_velocity_z)
                                           )
            self.timer.sleep()
            current_yaw = tf.transformations.euler_from_quaternion(
                                                                        [
                                                                            self.odometry.pose.pose.orientation.x,
                                                                            self.odometry.pose.pose.orientation.y,
                                                                            self.odometry.pose.pose.orientation.z,
                                                                            self.odometry.pose.pose.orientation.w
                                                                        ]
                                                                  )[2]
            self.timer.sleep()
            angle_turned = angle_turned + abs(abs(current_yaw) - abs(previous_yaw))
            previous_yaw = current_yaw
            if angle_turned > abs(radians):
                rospy.loginfo("final bearing is: %f" %current_yaw)
                break
        self.velocity_publisher.publish(Twist())
        self.timer.sleep()

if __name__ == "__main__":
    robot = Robot()
    robot()