#!/usr/bin/python
import wiringpi2 as wpi
from random import randint
import rospy
import time 
from geometry_msgs.msg import Twist

def init():
    rospy.init_node('base_driver', anonymous=True)
    pub = rospy.Publisher("TWIST_RANDOM", Twist, queue_size = 1)
    twistMake(pub)
    
def twistMake(pub):
    while True:
        twistData = Twist()
        twistData.linear.x = randint(-9, 9)
        twistData.linear.y = randint(-9, 9)
        twistData.linear.z = 0
        twistData.angular.x = 0
        twistData.angular.y = 0
        twistData.angular.z = randint(-9, 9)
        pub.publish(twistData)
        time.sleep(1)

init()
    