#!/usr/bin/env python
import wiringpi2 as wpi
import rospy
from std_msgs.msg import Float64

def init():
    wpi.wiringPiSetup()
    wpi.pinMode(5,0) #7,2,3,4
    rospy.init_node('encoder', anonymous=False)
    pub = rospy.Publisher('encoder', Float64, queue_size=10)
    vec_to_pwm(pub)
        
def vec_to_pwm(pub):
    while True:
        read = wpi.digitalRead(5)     
        pub.publish(read)      
    
if __name__ == '__main__':
    try:
        init()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown('encoder')
        pass