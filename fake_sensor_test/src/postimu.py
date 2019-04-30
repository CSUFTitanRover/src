#!/usr/bin/env python2.7

import rospy, time, signal
from math import floor
from fake_sensor_test.msg import imu

o = 180.00
a = 0.00
l = 0.00

imu_pub = rospy.Publisher('/imu', imu, queue_size=1)
rospy.init_node('imu')
i = imu()


def sigint_handler(signum, frame):
    print 'CTRL+C Pressed!'
    exit()

signal.signal(signal.SIGINT, sigint_handler)

def fakeIt():
    global o, a, l
    offset_o = 0.1
    offset_a = 3.05
    offset_l = 0.01
    while True:
        #fake IMU Posting data
        i.yaw = ((floor(o * 10)) / 10)
        i.pitch = ((floor(a * 100)) / 100)
        i.roll = ((floor(l * 100)) / 100)

        if o > 200 or o < 160:
            offset_o *= -1
        if a > 60 or a < -60:
            offset_a *= -1
        if l > 2 or l < -2:
            offset_l *= -1   

        o += offset_o
        a += offset_a
        l += offset_l
        imu_pub.publish(i)
        time.sleep(.2)

if __name__ == '__main__':
    fakeIt()
