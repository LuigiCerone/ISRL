#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import RPi.GPIO as GPIO
from time import sleep

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class Car:
    def __init__(self):
        rospy.init_node('raspberry_car', anonymous=True)

        rospy.Subscriber('/direction', String, self.direction_callback)

        # enable_pin, motor_pin_1, motor_pin_2, side.
        self.left_motor = Motor(26, 13, 21, "left")
        self.right_motor = Motor(12, 6, 20, "right")

    def direction_callback(self, msg):
        rospy.loginfo("Just received: {}".format(msg.data))
        self.act(msg.data)

    def act(self, direction):
        if direction == 'north':
            self.move_forward()
        elif direction == 'south':
            self.move_backward()
        elif direction == 'west':
            self.turn_left()
        elif direction == 'east':
            self.turn_right()
        elif direction == 'stop':
            self.stop()
        else:
            print('Case not considered: {}'.format(direction))

    def move_forward(self):
        self.left_motor.move_forward()
        self.right_motor.move_forward()
        # sleep(1)

    def move_backward(self):
        self.left_motor.move_backward()
        self.right_motor.move_backward()
        # sleep(1)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def turn_right(self):
        self.left_motor.move_forward()
        self.right_motor.move_backward()
        sleep(0.08)
        self.stop()

    def turn_left(self):
        self.left_motor.move_backward()
        self.right_motor.move_forward()
        sleep(0.08)
        self.stop()


class Motor:
    def __init__(self, enable_pin, motor_pin_1, motor_pin_2, side):

        GPIO.setup(enable_pin, GPIO.OUT)
        GPIO.setup(motor_pin_1, GPIO.OUT)
        GPIO.setup(motor_pin_2, GPIO.OUT)

        self.enable_pin = enable_pin
        self.motor_pin_1 = motor_pin_1
        self.motor_pin_2 = motor_pin_2
        self.side = side

    def move_forward(self, secs=1):
        rospy.loginfo("Motor {}: forward motion.".format(self.side))
        GPIO.output(self.motor_pin_1, GPIO.HIGH)
        GPIO.output(self.motor_pin_2, GPIO.LOW)
        GPIO.output(self.enable_pin, GPIO.HIGH)

        # sleep(secs)

    def move_backward(self, secs=1):
        rospy.loginfo("Motor {}: backward motion.".format(self.side))
        GPIO.output(self.motor_pin_1, GPIO.LOW)
        GPIO.output(self.motor_pin_2, GPIO.HIGH)
        GPIO.output(self.enable_pin, GPIO.HIGH)

        # sleep(secs)

    def stop(self):
        rospy.loginfo("Motor {} : stop.".format(self.side))
        GPIO.output(self.enable_pin, GPIO.LOW)


if __name__ == "__main__":
    car = Car()
    rospy.spin()
    GPIO.cleanup()


