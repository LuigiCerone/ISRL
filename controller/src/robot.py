import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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

        self.DISTANCE_THRESHOLD = 5 # what is the unit measure?

        # TODO Check if this is useful, in the previous hw this was used to avoid a bug with prothonics.
        self.previous_decision = None

        # Setup prothonics.
        self.prothonics = prothonics.Prothonics(100, 100)
        self.prothonics.useBrain().useLearning().learnKnoledgeBaseFromFile("behaviour.pl")
        # sub = rospy.Subscriber('scan', LaserScan, callback=blindRobot.useSense)

        self.quaternion = Quaternion()
        self.odometry = Odometry()
        self.laserscan = LaserScan()

        # Create publisher and subscriber.
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # self.rotate_by(360, 1)
        self.start()

    def laser_callback(self, laserscan):
        self.laserscan = laserscan

    def odom_callback(self, odometry):
        self.odometry = odometry

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
        
        while(abs(round(self.useQuaternion() % 360, 5) - abs(degree)) > 0.05):
            self.velocity_publisher.publish(command)

        command.angular.z = 0
        self.velocity_publisher.publish(command)

    def rotate_to_point(self, x, y, angular_velocity):
        print('guardo a nord')
        self.rotate_by(0, 0.05)
        rospy.sleep(5)
        delta_x = x - round(self.odometry.pose.pose.position.x, 7)
        delta_y = y - round(self.odometry.pose.pose.position.y, 7)
        theta_rads = math.atan2(delta_y, delta_x)
        theta = math.degrees(theta_rads)
        print('guardo verso il punto')
        self.rotate_by(round(theta,5), angular_velocity)

    def sense(self):
        # TODO Once working use 8 directions with range of angles.
        # In self.laserscan we have the reading of laser sensor.
        view = dict()
        if self.laserscan.ranges[0] <= self.DISTANCE_THRESHOLD:
            view['North'] = False # Because in prothonics logic false means there isn't an obstacle.

        if self.laserscan.ranges[89] <= self.DISTANCE_THRESHOLD:
            view['West'] = False
        # TODO Check if 89th element is left or right?

        if self.laserscan.ranges[159] <= self.DISTANCE_THRESHOLD:
            view['South'] = False

        if self.laserscan.ranges[269] <= self.DISTANCE_THRESHOLD:
            view['East'] = False

        return view

    def think(self, view):
        self.prothonics.useBrain().reactTo("perception(['{0}', '{1}', '{2}', '{3}'])".format(view[0], view[1], view[2],
                                                                                             view[3]), "takeDecision()")

        direction = None
        try:
            direction = self.prothonics.useBrain().useMemory().getAllDecisions()[-1][0]
            curr_decision = self.prothonics.useBrain().useMemory().getAllDecisions()[-1][1]
            if curr_decision == self.previous_decision:
                direction = None
            else:
                self.previous_decision = curr_decision
        except IndexError:
            direction = None
        return direction

    def act(self, direction):
        command = Twist()

        if direction == 'North':
            command.linear.x = 0.5
            command.angular.z = 0.0

        if direction == 'West':
            # Devo ruotarmi ed andare avanti oppure basta utilizzare command.linear.y oppure command.angular.y?
            pass

        if direction == 'South':
            command.linear.x = -0.5
            command.angular.z = 0.0

        if direction == 'East':
           pass

        self.velocity_publisher.publish(command)


    def start(self):
        rospy.loginfo("Robot started...")
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
        #rospy.loginfo("Robot moved")

        # TODO this should be a while true but for now we try only one move (that should be north).


if __name__ == "__main__":
    robot = Robot()
    rospy.spin()
