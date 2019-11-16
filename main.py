import rospy
from sensor_msgs.msg import LaserScan

def main():
    rospy.init_node('laser_data')
    sub = rospy.Subscriber('scan', LaserScan, read_scan)

    rospy.spin()

def read_scan(msg):
    print(msg.ranges[0])


if __name__ == '__main__':
    main()