import rospy
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO
from time import sleep


class Car:
    def __init__(self):
        rospy.init_node('raspberry_car', anonymous=True)

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Car setup.
        GPIO.setmode(GPIO.BOARD)

        # enable_pin, motor_pin_1, motor_pin_2.
        self.left_motor = Motor(22, 16, 18)
        self.right_motor = Motor(11, 15, 13)


    def cmd_vel_callback(self, cmd_vel):
        rospy.loginfo("Just received: {}".format(cmd_vel))

    def move_forward(self):
        self.left_motor.move_forward()
        self.right_motor.move_forward()
        self.stop()

    def move_backword(self):
        self.left_motor.move_backword()
        self.right_motor.move_backword()
        self.stop()

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

        GPIO.cleanup()


class Motor:
    def __init__(self, enable_pin, motor_pin_1, motor_pin_2):
        GPIO.setup(enable_pin, GPIO.OUT)
        GPIO.setup(motor_pin_1, GPIO.OUT)
        GPIO.setup(motor_pin_2, GPIO.OUT)

        self.enable_pin = enable_pin
        self.motor_pin_1 = motor_pin_1
        self.motor_pin_2 = motor_pin_2

    def move_forward(self, secs=3):
        rospy.loginfo("FORWARD MOTION")
        GPIO.output(self.motor_pin_1, GPIO.LOW)
        GPIO.output(self.motor_pin_2, GPIO.HIGH)
        GPIO.output(self.enable_pin, GPIO.HIGH)

        sleep(secs)

    def move_backword(self, secs=3):
        rospy.loginfo("BACKWARD MOTION")
        GPIO.output(self.motor_pin_1, GPIO.HIGH)
        GPIO.output(self.motor_pin_2, GPIO.LOW)
        GPIO.output(self.enable_pin, GPIO.HIGH)

        sleep(secs)

    def stop(self):
        rospy.loginfo("STOP")
        GPIO.output(self.enable_pin, GPIO.LOW)


if __name__ == "__main__":
    car = Car()
    rospy.spin()

