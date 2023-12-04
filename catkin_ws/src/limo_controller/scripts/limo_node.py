#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from pylimo import limo
import math

from adjust_speed import *

LIMO_ID = 1
NODE = "limo_node"

TOPIC_STATE = "/limo/state"
<<<<<<< HEAD
TOPIC_LIDAR = "/scan"
TOPIC_STOP = "/stop"
=======
TOPIC_LIDAR = "/scan_" + str(LIMO_ID)

>>>>>>> e6f88a0b01eb10c0addadb91937ebca0d971c3e8
QUEUE_SZ = 10
RATE_HZ = 5

MAX_SPEED = 1.0

DEBUG_STATE = True

STOP = False

def state_callback(msg: String):
    msg = str(msg)
    msg = msg.replace('"', '').replace("data: ", "")
    id, lin_vel, steer_angle = msg.split(";")
    
<<<<<<< HEAD
    '''
    isLeader = int(id) == (LIMO_ID - 1)
    if isLeader:
        print("Here: ", id, lin_vel, steer_angle)
        mylimo.SetMotionCommand(linear_vel=float(lin_vel), steering_angle=float(steer_angle))
    '''
=======
    isLeader = int(id) == (LIMO_ID - 1)
    if isLeader:
        mylimo.SetMotionCommand(linear_vel=float(lin_vel), steering_angle=float(steer_angle))
    
>>>>>>> e6f88a0b01eb10c0addadb91937ebca0d971c3e8
    if DEBUG_STATE:
        print("Received: ", id, lin_vel, steer_angle)

def stop_callback(stopVal: String):
    stopVal = str(stopVal)
    STOP = stopVal=="STOP"

    if DEBUG_STATE:
        print("STOP: ", STOP)
    
def getCurrentState(mylimo, id):
    lin_vel = mylimo.GetLinearVelocity()
    steer_angle = mylimo.GetSteeringAngle()
    return str(id) + ";" + str(lin_vel) + ";" + str(steer_angle)

if __name__ == '__main__':

    # Set up ros
    rospy.init_node(NODE + str(LIMO_ID))
    rospy.loginfo("Limo node " + NODE + str(LIMO_ID) + " has been started.")
    pub_state = rospy.Publisher(TOPIC_STATE, String, queue_size=QUEUE_SZ)
    print("Publishing to: ", TOPIC_STATE)
    if LIMO_ID != 0:
        print("Subscribing to: ", TOPIC_STATE, ", ", TOPIC_LIDAR)
        sub_state = rospy.Subscriber(TOPIC_STATE, String, callback=state_callback)
        sub_lidar = rospy.Subscriber(TOPIC_LIDAR, LaserScan, callback=scan_callback)
    rate = rospy.Rate(RATE_HZ)
    
    # Set up stop 
    pub_stop = rospy.Publisher(TOPIC_STOP, String, queue_size = 10)
    sub_stop = rospy.Subscriber(TOPIC_STOP, String, callback=stop_callback)
    # Set up limo
    mylimo = limo.LIMO()
    mylimo.EnableCommand()

    prevError = 0
    integral = 0
    while not rospy.is_shutdown():
        pub_state.publish(getCurrentState(mylimo, LIMO_ID))         
        adjustSpeed, error, integral = pid(RATE_HZ, prevError, integral)
        prevError = error
        newSpeed = mylimo.GetLinearVelocity() + adjustSpeed
        if STOP:
            newSpeed = 0
        elif newSpeed > MAX_SPEED:
            newSpeed = MAX_SPEED
        elif newSpeed < (-1 * MAX_SPEED):
            newSpeed = -1 * MAX_SPEED
        #mylimo.SetMotionCommand(linear_vel=float(newSpeed))

        rate.sleep()
