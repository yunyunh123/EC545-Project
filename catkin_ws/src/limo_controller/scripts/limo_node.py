#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from pylimo import limo

NODE = "limo_node"
LIMO_ID = 1

TOPIC_FOL = "/limo/follower_state"
TOPIC_LDR = "/limo/leader_state"
QUEUE_SZ = 10
RATE_HZ = 2
    

def leader_state_callback(msg: String):
    print(msg)
    #leader_state = str(msg).split(";")
    #ldr_id = leader_state[0].replace("data: ", "")#int(leader_state[0])
    #lin_vel = float(leader_state[1])
    #steer_angle = float(leader_state[2])
    #print(leader_state) #ldr_id, lin_vel, steer_angle)
    
def getCurrentState(mylimo, id):
    lin_vel = mylimo.GetLinearVelocity()
    ang_vel = mylimo.GetAngularVelocity()
    steer_angle = mylimo.GetSteeringAngle()
    return str(id) + ";" + str(lin_vel) + ";" + str(steer_angle)


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
        pub.publish(getCurrentState(mylimo, LIMO_ID))                
        rate.sleep()
