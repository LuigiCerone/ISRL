import os

if __name__ == '__main__':
    os.system("roscore")

    os.system("roslaunch turtlebot3_gazebo turtlebot3_world.launch")

    os.system("roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping")

    os.system("roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch")

    os.system("echo ciao")