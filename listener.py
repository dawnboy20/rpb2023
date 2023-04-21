#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

def callback(msg):
    print(msg)
    rospy.loginfo(msg)

def listener():
    sub = rospy.Subscriber('/turtle1/cmd_vel', Twist, callback)
    rospy.init_node('reader')
    rospy.spin()
#topic name is /turtle1/cmd_vel, topic type is Twist


if __name__ == '__main__':
    listener()
