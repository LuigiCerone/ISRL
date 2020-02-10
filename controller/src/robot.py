#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import tf
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
import prothonics

PI = 3.1415926535897


class Robot:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        self.home_x = None
        self.home_y = None

        self.DISTANCE_THRESHOLD = 0.2

        # TODO Check if this is useful, in the previous hw this was used to avoid a bug with prothonics.
        self.previous_decision = None
        self.previous_sense = None

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
        rospy.Subscriber("/scan_filtered", LaserScan, self.laser_callback)

        self.should_go_home = False
        rospy.Subscriber('/home', String, self.home_callback)

        self.compute_initial_pose()

        # For debugging only.
        # rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.move_base_callback)

        # self.get_home()
        self.init()
        # self.rotate_by(360, 1)
        # self.start()

    def laser_callback(self, laserscan):
        self.laserscan = laserscan

    def odom_callback(self, odometry):
        self.odometry = odometry

    def home_callback(self, msg):
        rospy.loginfo("Just received on topic /home the message: {}, I'm going home!".format(msg.data))
        self.should_go_home = True

    def move_base_callback(self, msg):
        rospy.loginfo("Eccolo {}".format(msg))

    def compute_initial_pose(self):
        self.initial_pose = rospy.wait_for_message('/pose', PoseStamped)
        rospy.loginfo("The default position is: {}".format(self.initial_pose))

        # Trasfrom from PoseStamped to MoveBaseActionGoal.

        move_base = MoveBaseGoal(self.initial_pose)

        move_base_action = MoveBaseActionGoal()
        move_base_action.goal = move_base
        move_base_action.goal.target_pose.header.frame_id = "map"
        move_base_action.header.stamp.secs = self.initial_pose.header.stamp.secs
        move_base_action.header.stamp.nsecs = self.initial_pose.header.stamp.nsecs
        move_base_action.header.frame_id = "map"

        self.goal = move_base_action

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

    def rotate_by(self, degree, angular_velocity):
        command = Twist()
        command.angular.z = angular_velocity

        starting_rad = round(self.useQuaternion() % 360, 5)

        while abs(round((round(starting_rad + abs(degree) % 360, 5) - abs(round(self.useQuaternion() % 360, 5))) % 360,
                        5)) \
                > 2:
            # print("differenza: {}".format(abs(round((round(starting_rad + abs(degree) % 360, 5) - abs(round(self.useQuaternion() % 360, 5))) % 360,           5))))
            # print("devo andare {}".format(abs(round((round(starting_rad + abs(degree) % 360, 5))))))
            print("attuale. {}".format(abs(round(self.useQuaternion() % 360, 5)) % 360))
            self.velocity_publisher.publish(command)

        command.angular.z = 0
        self.velocity_publisher.publish(command)

    def stop(self):
        command = Twist()
        command.linear.x = 0
        self.velocity_publisher.publish(command)

    def sense(self):
        # In self.laserscan we have the reading of laser sensor.
        view = dict()
        view['North'] = False  # In prothonics' logic False means there isn't an obstacle.
        count = 0

        t = 50
        # 90.
        for angle in range(0, 40):
            if self.laserscan.ranges[angle] <= self.DISTANCE_THRESHOLD or \
                    self.laserscan.ranges[(719 - angle) % 719] <= self.DISTANCE_THRESHOLD:
                count += 1

        if count >= (t/2):
            view['North'] = True

        count = 0
        view['West'] = False
        # for angle in range(90, 270):
        for angle in range(140, 220):
            if self.laserscan.ranges[angle] <= self.DISTANCE_THRESHOLD:
                count += 1

        if count >= t:
            view['West'] = True

        view['South'] = False
        count = 0
        # for angle in range(270, 450):
        for angle in range(320, 400):
            if self.laserscan.ranges[angle] <= self.DISTANCE_THRESHOLD:
                count += 1

        if count >= t:
            view['South'] = True

        view['East'] = False
        count = 0
        # for angle in range(450, 630):
        for angle in range(500, 580):
            if self.laserscan.ranges[angle] <= self.DISTANCE_THRESHOLD:
                count += 1

        if count >= t:
            view['East'] = True

        print(view)
        return view

    def think(self, view):
        self.prothonics.useBrain().reactTo("perception(['{}', '{}', '{}', '{}'])".format(view['North'], view['West'],
                                                                                         view['East'], view['South']),
                                           "takeDecision()")
        direction = None
        try:
            direction = self.prothonics.useBrain().useMemory().getAllDecisions()[-1][0]
        except IndexError:
            direction = None
        print(direction)
        return direction

    def act(self, direction):
        command = Twist()

        if direction == 'North':
            command.linear.x = 2.0
            command.angular.z = 0.0

        if direction == 'West':
            # self.rotate_by(89, 20.0)
            self.rotate(89, 20.0)
            # self.rotate_with_nav_stack(90)
            command.linear.x = 2.0

        if direction == 'South':
            # self.rotate_by(179, 20.0)
            self.rotate(179, 20.0)
            command.linear.x = 2.0

        if direction == 'East':
            # self.rotate_by(270, 20.0)
            self.rotate(269, 20.0)
            command.linear.x = 2.0

        self.velocity_publisher.publish(command)

    def rotate_with_nav_stack(self, degree):
        self.stop()

        goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

        move_base = MoveBaseGoal()

        val = tf.transformations.quaternion_from_euler(0, 0, degree)
        quat = Quaternion(
            x=val[0],
            y=val[1],
            z=val[2],
            w=val[3]
        )
        move_base.target_pose.pose.orientation = quat
        move_base_action = MoveBaseActionGoal()

        move_base_action.goal = move_base
        move_base_action.goal.target_pose.header.frame_id = "map"
        # move_base_action.header.stamp.secs = self.initial_pose.header.stamp.secs
        # move_base_action.header.stamp.nsecs = self.initial_pose.header.stamp.nsecs
        move_base_action.header.frame_id = "map"

        goal_publisher.publish(move_base_action)

    def rotate(self, angle, speed, clockwise=False):
        vel_msg = Twist()

        # Converting from angles to radians
        # angular_speed = speed * 2 * PI / 360
        angular_speed = speed
        relative_angle = angle * 2 * PI / 360

        # We wont use linear components
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        # Checking if our movement is CW or CCW

        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus

        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        self.velocity_publisher.publish(vel_msg)
        while current_angle < relative_angle:
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)

        # Forcing our robot to stop.
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def init(self):
        rospy.loginfo(":::INIT:::")
        goal_publisher = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)
        goal_publisher.publish(self.goal)
        rospy.loginfo("Ho scritto {}".format(self.goal))

    def go_north(self):
        flag = False
        count = 0
        while True:
            # self.prothonics.useBrain().useMemory().putNewFact("position({},{}).".format(
            # self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y))
            for angle in range(0, 40):
                if self.laserscan.ranges[angle] <= self.DISTANCE_THRESHOLD or \
                        self.laserscan.ranges[(719 - angle) % 719] <= self.DISTANCE_THRESHOLD:
                    count += 1

            if count >= 25:
                flag = True

            # for angle in range(0, 45):
            #     if self.laserscan.ranges[angle] <= self.DISTANCE_THRESHOLD or \
            #             self.laserscan.ranges[(360 - angle) % 360] <= self.DISTANCE_THRESHOLD:
            #         flag = True
            #         break
            if flag:
                break
        self.stop()
        print("Can't go north anymore...")

    def run(self):
        """The control loop of the car."""

        goal_sent = False
        rate = rospy.Rate(2)
        rospy.sleep(2)

        while not rospy.is_shutdown():
            if self.should_go_home and goal_sent == False:
                rospy.loginfo("Actually going home...")
                goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)

                goal_publisher.publish(self.goal)
                goal_sent = True
                self.should_go_home = True
                rospy.loginfo("Ho scritto {}".format(self.goal))

            elif not self.should_go_home:
                # flag = False
                # sense = self.sense()
                # if sense != self.previous_sense:
                #     print(sense)
                #     self.previous_sense = sense
                #     think = self.think(sense)
                #     # think = 'North'
                #     if think is not None:
                #         # print("vado a {}".format(think))
                #         self.act(think)
                #     else:
                #         break
                # else:
                #     # print('come prima')
                #     continue
                sense = self.sense()
                think = self.think(sense)
                if think is not None:
                    self.act(think)
                else:
                    break
                self.go_north()
            else:
                pass

            rate.sleep()


if __name__ == "__main__":
    robot = Robot()
    robot.run()
