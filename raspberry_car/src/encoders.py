#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
import RPi.GPIO as GPIO

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


class Encoder:
    def __init__(self, pin, side):
        self._pin = pin
        self._side = side

        self._count_pulses = 0
        GPIO.setup(pin, GPIO.IN)

        # Setup callback for GPIO.RISING/BOTH/FALLING
        GPIO.add_event_detect(self._pin, GPIO.FALLING)
        GPIO.add_event_callback(self._pin, self.update_count)

    def read(self):
        #read = GPIO.input(self._pin)
        #rospy.loginfo("side: {}; read: {}, count: {}".format(self._side, read, self._count_pulses))
        #if read:
        #    self._count_pulses += 1

        return self._count_pulses

    def update_count(self, channel):
        rospy.loginfo("side: {}; read: {}, count: {}".format(self._side, True, self._count_pulses))
        self._count_pulses += 1
        while GPIO.input(self._pin):
            pass


class Driver:
    def __init__(self):
        rospy.init_node('raspberry_car_encoder_driver')

        # self._last_received = rospy.get_time()
        # self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 1)
        self._max_speed = rospy.get_param('~max_speed', 10)
        self._wheel_base = rospy.get_param('~wheel_base', 0.091)

        # Assign pins to motors.
        self._left_encoder = Encoder(3, "left")
        self._right_encoder = Encoder(2, "right")

        # Setup subscriber for velocity messages.
        self._right_pub = rospy.Publisher('rwheel', Int16, queue_size=10)
        self._left_pub = rospy.Publisher('lwheel', Int16, queue_size=10)

    def run(self):
        """The control loop of the driver."""

        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            # Get new values.
            right_pulses = self._right_encoder.read()
            left_pulses = self._left_encoder.read()

            # Publish values.
            self._right_pub.publish(Int16(right_pulses))
            self._left_pub.publish(Int16(left_pulses))

            rate.sleep()


def main():
    driver = Driver()

    # Run driver.
    driver.run()


if __name__ == '__main__':
    main()
