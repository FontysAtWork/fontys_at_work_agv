#!/usr/bin/env python

## The driver of the AGV has 8 independant nodes (4 for writing the PWM parameter, and 4 to write the direction parameter).
## The reason for having individual nodes is due to the PWM's and encoder's having separate directories to write to and read from (hardware logic on the FPGA).
## With the os extension of python it is easier and more efficient to just cd to the right PWM and encoder once instead of switching continously.
## It is important that the data gets processed as quickly as possible to reduce the chance of overhead and putting an asynchronous delay on the individual wheels.
## When one wheel stutters or has a small overhead on its data, the odomotry will be damaged and the navigation accuracy reduces.
## Never the less, if you run a good navigation stack it would not be much of a problem. Except if you care about the milimeters, not the centimeters.
## What also is important to note; the data flow from /cmd_vel to the driver nodes needs to be controlled to reduce buffer problems. 
## The refresh rate of the overall driver is determind by the refreshrate of the /cmd_vel topic.
## It would be adviced to throttle the topic to a fixed 60 Hz.

# Author: B. Brussen
# Institute: Fontys Engineering University of Applied sciences
# Contact: b.brussen@student.fontys.nl

from subprocess import Popen
import os
import rospy
from std_msgs.msg import Int32

cmd_dir = '/sys/class/PWM/PWM1'

path = '/sys/class/PWM/PWM1/ENABLE'
d = os.open(path, os.O_WRONLY)

def init():
    os.chdir(cmd_dir)
    Popen('printf 0 > ENABLE', shell=True)
    rospy.init_node('dir_out_rf', anonymous=True)
    rospy.Subscriber('/direction_wheel_rf', Int32, vec_to_pwm)
       
def vec_to_pwm(data):
    global d
    os.write(d,str(data.data))
    
if __name__ == '__main__':
    try:
        init()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown('dir_out_rf')
        pass