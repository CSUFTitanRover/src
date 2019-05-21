#!/usr/bin/env python

'''
File:         roverESC.py
Authors:      Georden Grabuskie / Shripal Rawal / Timothy Parks
Emails:       ggrabuskie@csu.fullerton.edu / rawalshreepal000@gmail.com / parkstimothyj@gmail.com
Description:  sends movement commands to ESC's and updates telemetry data node
'''
import rospy, subprocess, sys
from multijoy.msg import MultiJoy
from mobility.msg import Status
from sensor_msgs.msg import Joy
'''# To import packages from different Directories
rootDir = subprocess.check_output('locate TitanRover2019 | head -1', shell=True).strip().decode('utf-8')
sys.path.insert(0, rootDir + '/build/resources/python-packages')
from pysaber import DriveEsc
import pyarm



# Instantiating The Class Object For PySabertooth
wheels = DriveEsc(128, "mixed")
armMix = DriveEsc(129, "notMixed")
'''
IDLE_TIMEOUT = 15 #seconds
#use actual button numbers instead of 0-indexed array
j1_b1, j1_b2, j1_b3, j1_b4, j1_b5, j1_b6, j1_b7, j1_b8, j1_b9, j1_b10, j1_b11, j1_b12
j1_a1, j1_a2, j1_a3, j1_a4, j1_a5, j1_a6

j2_b1, j2_b2, j2_b3, j2_b4, j2_b5, j2_b6, j2_b7, j2_b8, j2_b9, j2_b10, j2_b11, j2_b12
j2_a1, j2_a2, j2_a3, j2_a4, j2_a5, j2_a6


#comms source for reference (variables not used)
ERROR = -1
LOCAL = 0
GHZ = 1
MHZ = 2

#modes
PAUSE = -1 #LS + B3
IDLE = 0
MOBILITY = 1 #LS + B2
ARM = 2    #LS + B4
BOTH = 3  #R2 + B1

#instantiate publisher structure
telem = Status()
telem.source = -1
telem.mode = MOBILITY
telem.throttle = .3
telem.armAttached = True

#global variables
last_mode = telem.mode
last_active = last_throttle_change = 0
def setStop(): #just set all values on both joysticks to 0
    global j1_a1, j1_a2, j1_a3, j1_a4, j1_a5, j1_a6, j1_b1, j1_b2, j1_b3, j1_b4, j1_b5, j1_b6, j1_b7, j1_b8, j1_b9, j1_b10, j1_b11, j1_b12,\
        j2_a1, j2_a2, j2_a3, j2_a4, j2_a5, j2_a6, j2_b1, j2_b2, j2_b3, j2_b4, j2_b5, j2_b6, j2_b7, j2_b8, j2_b9, j2_b10, j2_b11, j2_b12
    j1_a1, j1_a2, j1_a3, j1_a4, j1_a5, j1_a6, j1_b1, j1_b2, j1_b3, j1_b4, j1_b5, j1_b6, j1_b7, j1_b8, j1_b9, j1_b10, j1_b11, j1_b12,\
        j2_a1, j2_a2, j2_a3, j2_a4, j2_a5, j2_a6, j2_b1, j2_b2, j2_b3, j2_b4, j2_b5, j2_b6, j2_b7, j2_b8, j2_b9, j2_b10, j2_b11, j2_b12,\
            j1, j4, j51, j52\
        = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

def setVals(joy_data):
    global j1_a1, j1_a2, j1_a3, j1_a4, j1_a5, j1_a6, j1_b1, j1_b2, j1_b3, j1_b4, j1_b5, j1_b6, j1_b7, j1_b8, j1_b9, j1_b10, j1_b11, j1_b12,\
        j2_a1, j2_a2, j2_a3, j2_a4, j2_a5, j2_a6, j2_b1, j2_b2, j2_b3, j2_b4, j2_b5, j2_b6, j2_b7, j2_b8, j2_b9, j2_b10, j2_b11, j2_b12, njoys
    njoys = joy_data.njoys
    j1_a1 = joy_data.joys[0].axes[0]
    j1_a2 = joy_data.joys[0].axes[1] 
    j1_a3 = joy_data.joys[0].axes[2] 
    j1_a4 = joy_data.joys[0].axes[3] 
    j1_a5 = joy_data.joys[0].axes[4] 
    j1_a6 = joy_data.joys[0].axes[5] 
    j1_b1 = joy_data.joys[0].buttons[0] 
    j1_b2 = joy_data.joys[0].buttons[1] 
    j1_b3 = joy_data.joys[0].buttons[2] 
    j1_b4 = joy_data.joys[0].buttons[3] 
    j1_b5 = joy_data.joys[0].buttons[4] 
    j1_b6 = joy_data.joys[0].buttons[5] 
    j1_b7 = joy_data.joys[0].buttons[6]
    j1_b8 = joy_data.joys[0].buttons[7] 
    j1_b9 = joy_data.joys[0].buttons[8] 
    j1_b10 = joy_data.joys[0].buttons[9] 
    j1_b11 = joy_data.joys[0].buttons[10]
    j1_b12 = joy_data.joys[0].buttons[11]
    j1 = int(j1_a5)
    j4 = int(j1_a6)
    j51 = (j1_b6, j1_b8)
    j52 = (j1_b5, j1_b7)

    if joy_data.njoys == 2:
        j2_a1 = joy_data.joys[1].axes[0]
        j2_a2 = joy_data.joys[1].axes[1] 
        j2_a3 = joy_data.joys[1].axes[2] 
        j2_a4 = joy_data.joys[1].axes[3] 
        j2_a5 = joy_data.joys[1].axes[4] 
        j2_a6 = joy_data.joys[1].axes[5] 
        j2_b1 = joy_data.joys[1].buttons[0] 
        j2_b2 = joy_data.joys[1].buttons[1] 
        j2_b3 = joy_data.joys[1].buttons[2] 
        j2_b4 = joy_data.joys[1].buttons[3] 
        j2_b5 = joy_data.joys[1].buttons[4] 
        j2_b6 = joy_data.joys[1].buttons[5] 
        j2_b7 = joy_data.joys[1].buttons[6]
        j2_b8 = joy_data.joys[1].buttons[7] 
        j2_b9 = joy_data.joys[1].buttons[8] 
        j2_b10 = joy_data.joys[1].buttons[9] 
        j2_b11 = joy_data.joys[1].buttons[10]
        j2_b12 = joy_data.joys[1].buttons[11]
        j1 = ((-1*j2_b3)+j2_b4)
        j4 = int(j2_a5)
        j51 = (j2_b5, j2_b6)
        j52 = (j2_b1, j2_b2)



def isActive():
    global j1_a1, j1_a2, j1_a3, j1_a4, j1_a5, j1_a6, j1_b1, j1_b2, j1_b3, j1_b4, j1_b5, j1_b6, j1_b7, j1_b8, j1_b9, j1_b10, j1_b11, j1_b12,\
        j2_a1, j2_a2, j2_a3, j2_a4, j2_a5, j2_a6, j2_b1, j2_b2, j2_b3, j2_b4, j2_b5, j2_b6, j2_b7, j2_b8, j2_b9, j2_b10, j2_b11, j2_b12
    if (abs(j1_a1) > 0 \
    or abs(j1_a2) > 0 \
    or abs(j1_a3) > 0 \
    or abs(j1_a4) > 0 \
    or abs(j1_a5) > 0 \
    or abs(j1_a6) > 0 \
    or abs(j2_a1) > 0 \
    or abs(j2_a2) > 0 \
    or abs(j2_a3) > 0 \
    or abs(j2_a4) > 0 \
    or abs(j2_a5) > 0 \
    or abs(j2_a6) > 0 \
    or (True in {j1_b1, j1_b2, j1_b3, j1_b4, j1_b5, j1_b6, j1_b7, j1_b8, j1_b9, j1_b10, j1_b11, j1_b12,\
        j2_b1, j2_b2, j2_b3, j2_b4, j2_b5, j2_b6, j2_b7, j2_b8, j2_b9, j2_b10, j2_b11, j2_b12})):
        return True
    else:
        return False

def main(data):
    global telem, last_active, last_throttle_change, last_mode, njoys
    global j1_a1, j1_a2, j1_a3, j1_a4, j1_a5, j1_a6, j1_b1, j1_b2, j1_b3, j1_b4, j1_b5, j1_b6, j1_b7, j1_b8, j1_b9, j1_b10, j1_b11, j1_b12,\
        j2_a1, j2_a2, j2_a3, j2_a4, j2_a5, j2_a6, j2_b1, j2_b2, j2_b3, j2_b4, j2_b5, j2_b6, j2_b7, j2_b8, j2_b9, j2_b10, j2_b11, j2_b12,\
            j1, j2, j51, j52

    setVals(data)
    telem.source = data.source

    #wake from idle or set mode to idle
    if isActive():
        last_active = data.header.stamp
        if telem.mode == IDLE:
            telem.mode = last_mode 
    elif (rospy.Time.now() - last_active) > rospy.Duration(IDLE_TIMEOUT):
        if (telem.mode != IDLE):
            last_mode = telem.mode
        telem.mode = IDLE
        telem_pub.publish(telem)

    #set mode
    if(j1_b9):
        if(j1_b3):
            telem.mode = PAUSE
        elif(j1_b2):
            telem.mode = MOBILITY
        elif(j1_b4):
            telem.mode = ARM
            setStop()
        elif(j1_b1):
            telem.mode = BOTH
        telem_pub.publish(telem)
    else:
        #single key presses for throttle
        if(j1_b4 and (telem.throttle < .95) and ((rospy.Time.now() - last_throttle_change) > rospy.Duration(0.25))):
            telem.throttle += 0.1
            last_throttle_change = rospy.Time.now()
        elif (j1_b2 and (telem.throttle > .25) and ((rospy.Time.now() - last_throttle_change) > rospy.Duration(0.25))):
            telem.throttle -= 0.1
            last_throttle_change = rospy.Time.now()
        telem_pub.publish(telem)
        try:
            if telem.mode in {MOBILITY, BOTH}:
                #turn in place
                if j1_b1:
                    wheels.driveBoth(0,-63)
                elif j1_b3:
                    wheels.driveBoth(0,63)
                else:
                    #normal movement
                    if telem.source is 3:
                        wheels.driveBoth(int(j1_a2),int(j1_a1))
                    else:
                        wheels.driveBoth(int(telem.throttle*127*j1_a2),int(-1 * telem.throttle*127*j1_a1))
            if njoys == 2 and telem.mode in {BOTH, ARM}:
                armMix.driveBoth(int(127*j2_a1),int(127*j2_a2))#j2, j3
                pyarm.armData(j1, j4, j51, j52) #j1, j4, j51, j52
            if telem.armAttached and telem.mode in {BOTH, ARM}:
                armMix.driveBoth(int(127*a3),int(127*a4))#j2, j3
                pyarm.armData(j1, j4, j51, j52) #j1, j1, j4, j51, j52
        except Exception as e:
            print("Mobility-main-drive error")
            print(e)

if __name__ == '__main__':
    try:
        setStop()
        rospy.init_node('rover_mobility', anonymous=True)
        last_active = last_throttle_change = rospy.Time.now()
        telem_pub = rospy.Publisher("telemetry", Status, queue_size=1)
        rospy.Subscriber("/multijoy", MultiJoy, main)

        rospy.spin() 
    except(KeyboardInterrupt, SystemExit):
        rospy.signal_shutdown("scheduled")
        raise

