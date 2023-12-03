#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from pylimo import limo
import math

from get_lidar_data import *

LIMO_ID = 1
NODE = "limo_node"

TOPIC_STATE = "/limo/state"
TOPIC_LIDAR = "/scan"

QUEUE_SZ = 10
RATE_HZ = 5
    
DEBUG_STATE = False

def state_callback(msg: String):
    msg = str(msg)
    msg = msg.replace('"', '').replace("data: ", "")
    id, lin_vel, steer_angle = msg.split(";")
    
    isLeader = int(id) == (LIMO_ID - 1)
    if isLeader:
        mylimo.SetMotionCommand(linear_vel=float(lin_vel), steering_angle=float(steer_angle))
    
    if DEBUG_STATE:
        print("Received: ", id, lin_vel, steer_angle)

def getCurrentState(mylimo, id):
    lin_vel = mylimo.GetLinearVelocity()
    steer_angle = mylimo.GetSteeringAngle()
    return str(id) + ";" + str(lin_vel) + ";" + str(steer_angle)

if __name__ == '__main__':

    # Set up ros
    rospy.init_node(NODE + str(LIMO_ID))
    rospy.loginfo("Limo node has been started.")
    pub_state = rospy.Publisher(TOPIC_STATE, String, queue_size=QUEUE_SZ)
    if LIMO_ID != 0:
        sub_state = rospy.Subscriber(TOPIC_STATE, String, callback=state_callback)
        sub_lidar = rospy.Subscriber(TOPIC_LIDAR, LaserScan, callback=scan_callback)
    rate = rospy.Rate(RATE_HZ)

    # Set up limo
    mylimo = limo.LIMO()
    mylimo.EnableCommand()

    while not rospy.is_shutdown():
        pub_state.publish(getCurrentState(mylimo, LIMO_ID))                
        rate.sleep()
