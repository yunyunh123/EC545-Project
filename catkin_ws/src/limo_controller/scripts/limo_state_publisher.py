#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

__DEBUG__ = True

NODE = "limo_state_publisher"
TOPIC = "/limo/state"
QUEUE_SZ = 10

if __name__ == '__main__':
    rospy.init_node(NODE)
    rospy.loginfo("Node has been started.")

    pub = rospy.Publisher("/limo/state", String, queue_size=QUEUE_SZ)
    
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        if __DEBUG__:
            pub.publish("Hello world!")
        #else:
            
        rate.sleep()