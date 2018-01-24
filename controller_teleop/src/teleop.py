#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

speed_factor = float(1)

def callback(data):
    global speed_factor
    button_A = data.buttons[0]    
    if button_A == 1:
        speed_factor = 0
        print("Maximum speed = " + str(speed_factor))
    button_LB = data.buttons[4]
    if button_LB == 1:
        speed_factor -= 0.1
        print("Maximum speed = " + str(speed_factor))
    button_RB = data.buttons[5]
    if button_RB == 1:
        speed_factor += 0.1
        print("Maximum speed = " + str(speed_factor))
        
    twist = Twist()
    
    if data.axes[1] < 0.1 and data.axes[1] > -0.1:
        twist.linear.x = 0
    else:
        twist.linear.x = speed_factor*data.axes[1]
        
    if data.axes[0] < 0.1 and data.axes[0] > -0.1:
        twist.linear.y = 0
    else: 
        twist.linear.y = speed_factor*data.axes[0]
        
    if data.axes[3] < 0.1 and data.axes[3] > -0.1:
        twist.angular.z = 0
    else:
        twist.angular.z = speed_factor*data.axes[3] * 1.2
        
    pub.publish(twist)
        
# Intializes everything
def start():
    
    # publishing to "/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 0)
    
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy_throttle", Joy, callback)
    # starts the node
    rospy.init_node('controller_teleop')
    rospy.spin()
    
if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown('controller_teleop')
        pass