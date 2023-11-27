#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

NODE = "limo_state_subscriber"
TOPIC = "/limo/state"

def state_callback(msg: String):
    rospy.loginfo(msg)

if __name__ == '__main__':
    rospy.init_node(NODE)
    rospy.loginfo("Node has been started.")

    sub = rospy.Subscriber(TOPIC, String, callback=state_callback)

    rospy.spin()