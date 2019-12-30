#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

import RPi.GPIO as GPIO


# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

_FREQUENCY = 500


def _clip(value, minimum, maximum):
    """Ensure value is between minimum and maximum."""

    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value


class Motor:
    def __init__(self, activation_pin, motor_pin_1, motor_pin_2):
        self._activation_pin = activation_pin
        self._motor_pin_1 = motor_pin_1
        self._motor_pin_2 = motor_pin_2

        GPIO.setup(activation_pin, GPIO.OUT)
        GPIO.setup(motor_pin_1, GPIO.OUT)
        GPIO.setup(motor_pin_2, GPIO.OUT)

        self._activation_pin = GPIO.PWM(activation_pin, _FREQUENCY)
        self._activation_pin.start(activation_pin)

    def move(self, speed_percent):
        speed = _clip(abs(speed_percent), 0, 100)

        rospy.loginfo("speed: {}; speed_percent: {}".format(speed, speed_percent))

        # Positive speeds move wheels forward, negative speeds
        # move wheels backward
        if speed_percent < 0:
            GPIO.output(self._motor_pin_1, GPIO.LOW)
            GPIO.output(self._motor_pin_2, GPIO.HIGH)

            self._activation_pin.ChangeDutyCycle(speed)
            # GPIO.output(self._activation_pin, GPIO.HIGH)
        else:
            GPIO.output(self._motor_pin_1, GPIO.HIGH)
            GPIO.output(self._motor_pin_2, GPIO.LOW)

            self._activation_pin.ChangeDutyCycle(speed)
            # GPIO.output(self._activation_pin, GPIO.HIGH)


class Driver:
    def __init__(self):
        rospy.init_node('raspberry_car_motor_driver')

        # self._last_received = rospy.get_time()
        # self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 5)
        self._max_speed = rospy.get_param('~max_speed', 10)
        self._wheel_base = rospy.get_param('~wheel_base', 0.091)

        # Assign pins to motors.
        GPIO.setup(18, GPIO.IN, GPIO.PUD_UP)
        self._left_motor = Motor(26, 13, 21)
        self._right_motor = Motor(12, 6, 20)
        self._left_speed_percent = 0
        self._right_speed_percent = 0

        # Setup subscriber for velocity messages.
        rospy.Subscriber('rwheel_vtarget', Float32, self.right_wheel_velocity_callback)
        rospy.Subscriber('lwheel_vtarget', Float32, self.left_wheel_velocity_callback)

    def right_wheel_velocity_callback(self, message_speed):
        self._right_speed_percent = (100 * message_speed.data/self._max_speed)
        self._right_motor.move(self._right_speed_percent)
        # rospy.loginfo("right wheel, ho ricevuto dal topic {}, andro a {}.".format(message_speed.data, self._right_speed_percent))

    def left_wheel_velocity_callback(self, message_speed):
        self._left_speed_percent = (100 * message_speed.data/self._max_speed)
        self._left_motor.move(self._left_speed_percent)

    def run(self):
        """The control loop of the driver."""

        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            self._left_motor.move(self._left_speed_percent)
            self._right_motor.move(self._right_speed_percent)
            rate.sleep()

    def run1(self):
        """The control loop of the driver."""
        rospy.spin()


def main():
    driver = Driver()

    # Run driver. This will block
    driver.run1()


if __name__ == '__main__':
    main()
