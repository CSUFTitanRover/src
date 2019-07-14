#!/usr/bin/env python2.7

import rospy, time, signal
from fake_sensor_test.msg import gps

curr_pos = [33.881317,-117.882611]
percision_lat = 0.000001
percision_lon = -0.000001

gps_pub = rospy.Publisher('/gnss', gps, queue_size=1)
rospy.init_node('gnss')
g = gps()

g.baseLat, g.baseLon = '33.881832' , '-117.882965'
g.destLat, g.destLon = '33.881650' , '-117.883149'

def sigint_handler(signum, frame):
    print 'CTRL+C Pressed!'
    exit()

signal.signal(signal.SIGINT, sigint_handler)

def fakeIt():
    global curr_pos, percision_lat, percision_lon
    while True:
        #fake GPS Posting data
        if curr_pos[0] > 33.881620 or curr_pos[0] < 33.881317:
            percision_lat *= -1
        if curr_pos[1] > -117.882611 or curr_pos[1] < -117.883346:
            percision_lon *= -1 
        curr_pos = (curr_pos[0] + percision_lat, curr_pos[1] + percision_lon)
	g.roverLat, g.roverLon = str(curr_pos[0]), str(curr_pos[1])
	gps_pub.publish(g)
        time.sleep(.2)

if __name__ == '__main__':
    fakeIt()
