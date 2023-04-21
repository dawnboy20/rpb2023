#!/usr/bin/env python3
# license removed for brevity
from std_msgs.msg import String
import rospy
import subprocess
import time


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=20)
    rospy.init_node('talker')
    rate = rospy.Rate(10)
    time.sleep(0.1)

    hello_str = "hello word %s" % rospy.get_time()
    for i in range(10):
        pub.publish(hello_str)

        

   
 

    
# TODO: publish 10 string data, topic name is chatter




    
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
