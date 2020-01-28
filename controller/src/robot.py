#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import tf
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
import prothonics


class Robot:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        self.home_x = None
        self.home_y = None

        self.DISTANCE_THRESHOLD = .3 # what is the unit measure?

        # TODO Check if this is useful, in the previous hw this was used to avoid a bug with prothonics.
        self.previous_decision = None

        # Setup prothonics.
        self.prothonics = prothonics.Prothonics(100, 100)
        self.prothonics.useBrain().useLearning().learnKnoledgeBaseFromFile(rospy.get_param("~prolog"))
        # sub = rospy.Subscriber('scan', LaserScan, callback=blindRobot.useSense)

        self.quaternion = Quaternion()
        self.odometry = Odometry()
        self.laserscan = LaserScan()

        # Create publisher and subscriber.
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # Note: True means don't go home.
        self.should_go_home = True
        rospy.Subscriber('/home', String, self.home_callback)

        self.initial_pose = self.compute_initial_pose()
        # self.rotate_by(360, 1)
        # self.start()

    def laser_callback(self, laserscan):
        self.laserscan = laserscan

    def odom_callback(self, odometry):
        self.odometry = odometry
    
    def home_callback(self, msg):
        rospy.loginfo("Just received on topic /home the message: {}, I'm going home!".format(msg.data))
        self.should_go_home = False

    def compute_initial_pose(self):
        # The following should compute the pose, it's commented out beacuse I've used a package to do so.
        # listener = tf.TransformListener()
        # pose_stamped = PoseStamped()

        # try:
        #     now = rospy.Time.now()
        #     listener.waitForTransform('map', 'base_link', now, rospy.Duration(10.0))
        #     (bf_trans, bf_rot) = listener.lookupTransform('map', 'base_link', now)
        #     # [ x, y, z ] [x, y, z, w]
        #     pose_stamped.header.stamp = rospy.Time.now()
        #     pose_stamped.header.frame_id = "tf/map"

        #     pose_stamped.pose.position.x = bf_trans[0]
        #     pose_stamped.pose.position.y = bf_trans[1]
        #     pose_stamped.pose.position.z = bf_trans[2]

        #     pose_stamped.pose.orientation.x = bf_rot[0]
        #     pose_stamped.pose.orientation.y = bf_rot[1]
        #     pose_stamped.pose.orientation.z = bf_rot[2]
        #     pose_stamped.pose.orientation.w = bf_rot[3]

        #     rospy.loginfo("Initial position: {}".format(pose_stamped.serialize()))

        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        #     rospy.logerr(e)
        # self.initial_pose = pose_stamped;
           
        self.initial_pose = rospy.wait_for_message('/pose', PoseStamped)
        rospy.loginfo("The default position is: {}".format(self.initial_pose))

    def useQuaternion(self):
        self.quaternion = Quaternion(
            x=self.odometry.pose.pose.orientation.x, 
            y=self.odometry.pose.pose.orientation.y,
            z=self.odometry.pose.pose.orientation.z,
            w=self.odometry.pose.pose.orientation.w
            )
        actual_m = tf.transformations.quaternion_matrix([self.quaternion.x, self.quaternion.y,
             self.quaternion.z, self.quaternion.w])
        actual_x = actual_m[0, 0]
        actual_y = actual_m[1, 0]
        actual_theta_rads = math.atan2(actual_y, actual_x)
        return math.degrees(actual_theta_rads)
        

    def move_to(self, x, y):
        print('mi aggiusto')
        self.rotate_to_point(x, y, 0.05)
        print('aspetto 5 sec e parto')
        rospy.sleep(5)
        command = Twist()
        while True:
            eucl_dist = math.sqrt(math.pow(round(self.odometry.pose.pose.position.x, 7) - x,2) + math.pow(round(self.odometry.pose.pose.position.y, 7) - y, 2))
            
            if(eucl_dist <= 0.1):
                break
            command.linear.x = 0.2
            command.angular.z = 0
            self.velocity_publisher.publish(command)
        
        command.linear.x = 0
        self.velocity_publisher.publish(command)

    def get_home(self):
        rospy.sleep(5)
        self.home_x = round(self.odometry.pose.pose.position.x, 7)
        self.home_y = round(self.odometry.pose.pose.position.y, 7)

    def rotate_by(self, degree, angular_velocity):
        command = Twist()
        command.angular.z = angular_velocity

        starting_rad = round(self.useQuaternion() % 360, 5)

        while abs(round((round(starting_rad + abs(degree) % 360, 5) - abs(round(self.useQuaternion() % 360, 5))) % 360, 5))\
                > 0.5:
            self.velocity_publisher.publish(command)

        command.angular.z = 0
        self.velocity_publisher.publish(command)

    def rotate_to_point(self, x, y, angular_velocity):
        print('guardo a nord')
        self.rotate_by(0, 0.2)
        rospy.sleep(5)
        delta_x = x - round(self.odometry.pose.pose.position.x, 7)
        delta_y = y - round(self.odometry.pose.pose.position.y, 7)
        theta_rads = math.atan2(delta_y, delta_x)
        theta = math.degrees(theta_rads)
        print('guardo verso il punto')
        self.rotate_by(round(theta,5), angular_velocity)

    def stop(self):
        command = Twist()
        command.linear.x = 0
        self.velocity_publisher.publish(command)

    def sense(self):
        # TODO Once working use 8 directions with range of angles.
        # In self.laserscan we have the reading of laser sensor.
        view = dict()
        view['North'] = False # In prothonics' logic False means there isn't an obstacle.
        for angle in range(0, 45):
            if self.laserscan.ranges[angle] <= self.DISTANCE_THRESHOLD or \
                    self.laserscan.ranges[(360 - angle) % 360] <= self.DISTANCE_THRESHOLD:
                view['North'] = True
        view['West'] = False
        for angle in range(45, 135):
            if self.laserscan.ranges[angle] <= self.DISTANCE_THRESHOLD:
                view['West'] = True
        # TODO Check if 89th element is left or right?
        view['South'] = False
        for angle in range(135, 225):
            if self.laserscan.ranges[angle] <= self.DISTANCE_THRESHOLD:
                view['South'] = True
        view['East'] = False
        for angle in range(225, 315):
            if self.laserscan.ranges[angle] <= self.DISTANCE_THRESHOLD:
                view['East'] = True

        print(view)
        return view

    def think(self, view):
        self.prothonics.useBrain().reactTo("perception(['{}', '{}', '{}', '{}'])".format(view['North'], view['West'],
                                                                     view['East'], view['South']), "takeDecision()")

        direction = None
        try:
            direction = self.prothonics.useBrain().useMemory().getAllDecisions()[-1][0]
            # Cosa contiene curr_decision?
            curr_decision = self.prothonics.useBrain().useMemory().getAllDecisions()[-1][1]
            
        except IndexError:
            direction = None
        print(direction)
        return direction

    def act(self, direction):
        command = Twist()

        if direction == 'North':
            command.linear.x = 0.5
            command.angular.z = 0.0

        if direction == 'West':
            self.rotate_by(89, 0.2)
            command.linear.x = 0.5

        if direction == 'South':
            self.rotate_by(179, 0.2)
            command.linear.x = 0.5

        if direction == 'East':
           self.rotate_by(269, 0.2)
           command.linear.x = 0.5

        self.velocity_publisher.publish(command)

    def start(self):
        '''rospy.loginfo("Robot started...")
        self.get_home()
        print('home taken')
        print('coordinate casa - x:{} , y:{}'.format(self.home_x, self.home_y))
        self.move_to(-2.00003, -0.99997)
        rospy.sleep(5)
        print('moved to 0.50 -0.50')
        print('current_position: {}, {}'.format(round(self.odometry.pose.pose.position.x,5), round(self.odometry.pose.pose.position.y,5)))
        print('going home')
        self.move_to(self.home_x, self.home_y)
        print('returned home - x: {}, y: {}'.format(round(self.odometry.pose.pose.position.x,5), round(self.odometry.pose.pose.position.y,5)))
        #view = self.sense()
        #rospy.loginfo("Current view for the controller is: '{0}', '{1}', '{2}', '{3}'.".format(view[0], view[1], view[2],
                                                                                             #view[3]))

        #direction = self.think(view)
        #rospy.loginfo("Chosen direction is: {}.".format(direction))

        #self.act(direction)
        #rospy.loginfo("Robot moved")'''
        rospy.sleep(2)
        while True:
            flag = False
            sense = self.sense()
            think = self.think(sense)
            if think is not None:
                self.act(think)
            else:
                break
                # TODO Siamo sicuri break e non continue? Da testare.
            while True:
                self.prothonics.useBrain().useMemory().putNewFact("position({},{}).".format(
                    self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y))
                for angle in range(0, 45):
                    if self.laserscan.ranges[angle] <= self.DISTANCE_THRESHOLD or \
                            self.laserscan.ranges[(360 - angle) % 360] <= self.DISTANCE_THRESHOLD:
                        flag = True
                        break
                if flag:
                    break
            self.stop()
            print("mi calibro")

    def run(self):
        """The control loop of the car."""

        rate = rospy.Rate(2)

        while not rospy.is_shutdown() and self.should_go_home:
            flag = False
            sense = self.sense()
            think = self.think(sense)
            if think is not None:
                self.act(think)
            else:
                break
            while True:
                self.prothonics.useBrain().useMemory().putNewFact("position({},{}).".format(
                    self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y))
                for angle in range(0, 45):
                    if self.laserscan.ranges[angle] <= self.DISTANCE_THRESHOLD or \
                            self.laserscan.ranges[(360 - angle) % 360] <= self.DISTANCE_THRESHOLD:
                        flag = True
                        break
                if flag:
                    break
            self.stop()
            print("mi calibro")

            rate.sleep()
        
        if self.should_go_home:
            rospy.loginfo("Actually going home...")
            # TODO Go home! In order to do this we need to pusblish on /move_base_simple/goal the inizial position in format PoseStamped.
            goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
            goal_publisher.publish(self.initial_pose)
    
if __name__ == "__main__":
    robot = Robot()
    # rospy.spin()
    robot.run()