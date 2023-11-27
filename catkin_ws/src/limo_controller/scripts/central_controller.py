#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

__DEBUG__ = True

NODE = "central_controller"

TOPIC_FOL = "/limo/follower_state"
TOPIC_LDR = "/limo/leader_state"
QUEUE_SZ = 10
RATE_HZ = 2

def follower_state_callback(msg: String):
    rospy.loginfo(msg)

    if __DEBUG__:
        pub.publish("Hello limo node!")

if __name__ == '__main__':
    rospy.init_node(NODE)
    rospy.loginfo("Central controller node has been started.")

    pub = rospy.Publisher(TOPIC_LDR, String, queue_size=QUEUE_SZ)
    sub = rospy.Subscriber(TOPIC_FOL, String, callback=follower_state_callback)

    rospy.spin()