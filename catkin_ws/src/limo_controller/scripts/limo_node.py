#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from pylimo import limo

LIMO_ID = 1

NODE = "limo_node"
TOPIC = "/limo/state"
QUEUE_SZ = 10
RATE_HZ = 2
    
def state_callback(msg: String):
    print("Received: ", msg)

def getCurrentState(mylimo, id):
    lin_vel = mylimo.GetLinearVelocity()
    steer_angle = mylimo.GetSteeringAngle()
    return str(id) + ";" + str(lin_vel) + ";" + str(steer_angle)

if __name__ == '__main__':

    # Set up ros
    rospy.init_node(NODE)
    rospy.loginfo("Limo node has been started.")
    pub = rospy.Publisher(TOPIC, String, queue_size=QUEUE_SZ)
    sub = rospy.Subscriber(TOPIC, String, callback=state_callback)
    rate = rospy.Rate(RATE_HZ)

    # Set up limo
    mylimo = limo.LIMO()
    mylimo.EnableCommand()

    while not rospy.is_shutdown():
        pub.publish(getCurrentState(mylimo, LIMO_ID))                
        rate.sleep()
