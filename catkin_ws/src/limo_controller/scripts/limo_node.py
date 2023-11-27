#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from pylimo import limo

__DEBUG__ = True

NODE = "limo_node"

TOPIC_FOL = "/limo/follower_state"
TOPIC_LDR = "/limo/leader_state"
QUEUE_SZ = 10
RATE_HZ = 2
    

def leader_state_callback(msg: String):
    rospy.loginfo(msg)

def getCurrentState(mylimo):
    lin_vel = mylimo.GetLinearVelocity()
    ang_vel = mylimo.GetAngularVelocity()
    steer_angle = mylimo.GetSteeringAngle()
    return str(lin_vel) + ";" + str(steer_angle)


if __name__ == '__main__':
    # Set up ros
    rospy.init_node(NODE)
    rospy.loginfo("Limo node has been started.")
    pub = rospy.Publisher(TOPIC_FOL, String, queue_size=QUEUE_SZ)
    sub = rospy.Subscriber(TOPIC_LDR, String, callback=leader_state_callback)
    rate = rospy.Rate(RATE_HZ)

    # Set up limo
    mylimo = limo.LIMO()
    mylimo.EnableCommand()

    while not rospy.is_shutdown():
        if __DEBUG__:
            pub.publish("Hello central controller!")
        else:
            pub.publish(getCurrentState(mylimo))
            
            
        rate.sleep()