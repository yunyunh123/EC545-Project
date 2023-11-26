#!/usr/bin/env python3
import rospy

NODE = "limo_state_publisher"
TOPIC = "/limo/state"
QUEUE_SZ = 10

if __name__ == '__main__':
    rospy.init_node(NODE)
    rospy.loginfo("Node has been started.")

    pub = rospy.Publisher("/limo/state", str, QUEUE_SZ)
    
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        pub.publish("Hello world!")
        rate.sleep()