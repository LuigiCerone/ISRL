import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf
import numpy as np
from geometry_msgs.msg import Quaternion
import prothonics

# Setup prothonics.
#blindRobot = prothonics.Prothonics(100, 100)
#blindRobot.useBrain().useLearning().learnKnoledgeBaseFromFile("behaviour.pl")
#sub = rospy.Subscriber('scan', LaserScan, callback=blindRobot.useSense)

actual_theta_degs = 0

def get_rotation(msg):
    global actual_theta_degs
    orientation_q = msg.pose.pose.orientation
    quaternion_0 = Quaternion(x=orientation_q.x, y=orientation_q.y, \
         z=orientation_q.z, w=orientation_q.w)
    actual_m = tf.transformations.quaternion_matrix([quaternion_0.x, quaternion_0.y, \
             quaternion_0.z, quaternion_0.w])
    actual_x = actual_m[0, 0]
    actual_y = actual_m[1, 0]
    actual_theta_rads = math.atan2(actual_y, actual_x)
    actual_theta_degs = actual_theta_rads * 180 / math.pi if actual_theta_rads * 180 / math.pi >= 0 else 360 + actual_theta_rads * 180 / math.pi

rospy.init_node('robot')
 
sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

command = Twist()
command.angular.z = 0.5

while(round(actual_theta_degs % 360) != 0):
    pub.publish(command)
command.angular.z = 0
pub.publish(command)
rospy.spin()