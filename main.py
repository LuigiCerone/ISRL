import rospy
from sensor_msgs.msg import LaserScan
import prothonics

def main():

    # Setup prothonics.
    blindRobot = prothonics.Prothonics(100, 100)
    blindRobot.useBrain().useLearning().learnKnoledgeBaseFromFile("behaviour.pl")

    # Create a node.
    rospy.init_node('laser_data')
    sub = rospy.Subscriber('scan', LaserScan, callback=blindRobot.useSense)

    rospy.spin()


if __name__ == '__main__':
    main()