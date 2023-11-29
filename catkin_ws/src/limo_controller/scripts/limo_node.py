#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from pylimo import limo

LIMO_ID = 0
NODE = "limo_node"

TOPIC = "/limo/state"
QUEUE_SZ = 10
RATE_HZ = 5
    
def state_callback(msg: String):
    msg = str(msg)
    msg = msg.replace('"', '').replace("data: ", "")
    id, lin_vel, steer_angle = msg.split(";")
    
    isLeader = int(id) == (LIMO_ID - 1)
    if isLeader:
        mylimo.SetMotionCommand(linear_vel=float(lin_vel), steering_angle=float(steer_angle))
    print("Received: ", id, lin_vel, steer_angle)

def getCurrentState(mylimo, id):
    lin_vel = mylimo.GetLinearVelocity()
    steer_angle = mylimo.GetSteeringAngle()
    return str(id) + ";" + str(lin_vel) + ";" + str(steer_angle)

if __name__ == '__main__':

    # Set up ros
    rospy.init_node(NODE + str(LIMO_ID))
    rospy.loginfo("Limo node has been started.")
    pub = rospy.Publisher(TOPIC, String, queue_size=QUEUE_SZ)
    if LIMO_ID != 0:
        sub = rospy.Subscriber(TOPIC, String, callback=state_callback)
    rate = rospy.Rate(RATE_HZ)

    # Set up limo
    mylimo = limo.LIMO()
    mylimo.EnableCommand()

    while not rospy.is_shutdown():
        pub.publish(getCurrentState(mylimo, LIMO_ID))                
        rate.sleep()
