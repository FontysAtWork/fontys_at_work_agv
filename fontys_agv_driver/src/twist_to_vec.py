#!/usr/bin/env python

### Below is the matlab code that has been used to calcuate the angular 
### velocities of the wheels. You can check if the mathimatical model is 
### correct and play with it yourself.

## MATLAB algorithm ##
#clc
#clear
#
#w1 = 0;
#w2 = 0;
#w3 = 0;
#w4 = 0;
#
#vx = 1;
#vy = 1;
#w0 = 0;
#
#L1 = 0.2;
#L2 = 0.3;
#R = 0.075;
#a = 45;
#
#twist = [vx vy w0];
#Lt = L1 + L2/tan(a);
#
#direction1 = [1 1/tan(a) -Lt];
#direction2 = [1 -1/tan(a) Lt];
#direction3 = [1 -1/tan(a) -Lt];
#direction4 = [1 1/tan(a) Lt];
#
#direction = [direction1;direction2;direction3;direction4];
#w = (1/R).*direction.*twist;
#
#for i = 1:3
#    w1 = w1 + w(1,i);
#    w2 = w2 + w(2,i);
#    w3 = w3 + w(3,i);
#    w4 = w4 + w(4,i);
#end 

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int32

def settings():
    L1 = 0.2;
    L2 = 0.3;
    R = 0.075;
    settings = [L1, L2, R]
    return settings

# Creates publishers for communication towards the driver
def publishers():
    pub1 = rospy.Publisher('vec_wheel_lf', Float64, queue_size=5)
    pub2 = rospy.Publisher('vec_wheel_rf', Float64, queue_size=5)
    pub3 = rospy.Publisher('vec_wheel_lb', Float64, queue_size=5)
    pub4 = rospy.Publisher('vec_wheel_rb', Float64, queue_size=5)
    pub5 = rospy.Publisher('direction_wheel_lf', Int32, queue_size=5)
    pub6 = rospy.Publisher('direction_wheel_rf', Int32, queue_size=5)
    pub7 = rospy.Publisher('direction_wheel_lb', Int32, queue_size=5)
    pub8 = rospy.Publisher('direction_wheel_rb', Int32, queue_size=5)
    publishers = [pub1, pub2, pub3, pub4, pub5, pub6, pub7, pub8]
    return publishers

def init():
    rospy.init_node('twist_to_vec', anonymous=True)
    
def listen():
    rospy.Subscriber("/cmd_vel", Twist, twist_to_vec)
    
# Main function that calculates the angular velocity of each individual wheel
def twist_to_vec(data):
    global settings
    global publishers
    
    # First the twist message needs to be interpreted
    twist_vx = data.linear.x
    twist_vy = data.linear.y
    twist_w0 = data.angular.z
    
    # The twist data is been put in an array. Array calculations make less code.
    twist_vec = np.array([[twist_vx], [twist_vy], [twist_w0]])
    Lt = settings[0] + settings[1]/np.tan(45)
    A = 1 / np.tan(45)
    
    # This part deines what the velocity is over the X and Y axis of the wheel.
    angular_vel_1 = [1, A, -Lt]
    angular_vel_2 = [1, -A, Lt]
    angular_vel_3 = [1, -A, -Lt]
    angular_vel_4 = [1, A, Lt]
    
    angular_vel = np.array([angular_vel_1, angular_vel_2, angular_vel_3, angular_vel_4])
    
    # Now the individual speed vectors are clculated. By deviding by the radius of the wheel we get the angular velocity.
    w = np.dot(angular_vel, twist_vec)
    angular_vel_1 = w[0].item() / settings[2]
    angular_vel_2 = w[1].item() / settings[2]
    angular_vel_3 = w[2].item() / settings[2]
    angular_vel_4 = w[3].item() / settings[2]

    # With the polrity of the velocities we can determine what direction the wheel should be spinning (3 for CW, 2 for CCW).
    # The PWM driver require a direction value of either 0 (off), 2 (CCW), 3 (CW).
    if angular_vel_1 > 0:
        direction_1 = 3
    elif angular_vel_1 < 0:
        direction_1 = 2
        angular_vel_1 = angular_vel_1 * -1
    else:
        direction_1 = 0
        
    if angular_vel_2 > 0:
        direction_2 = 2
    elif angular_vel_2 < 0:
        direction_2 = 3
        angular_vel_2 = angular_vel_2 * -1
    else:
        direction_2 = 0
        
    if angular_vel_3 > 0:
        direction_3 = 3
    elif angular_vel_3 < 0:
        direction_3 = 2
        angular_vel_3 = angular_vel_3 * -1
    else:
        direction_3 = 0
        
    if angular_vel_4 > 0:
        direction_4 = 2
    elif angular_vel_4 < 0:
        direction_4 = 3
        angular_vel_4 = angular_vel_4 * -1
    else:
        direction_4 = 0
        
    publishers[4].publish(direction_1)
    publishers[5].publish(direction_2)
    publishers[6].publish(direction_3)
    publishers[7].publish(direction_4)
    
    publishers[0].publish(angular_vel_1)
    publishers[1].publish(angular_vel_2)
    publishers[2].publish(angular_vel_3)
    publishers[3].publish(angular_vel_4)
    
    r.sleep()
    
if __name__ == '__main__':
    try:
        settings = settings()
        publishers = publishers()
        init()
        r = rospy.Rate(60)
        listen()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown('twist_to_vec')
        pass