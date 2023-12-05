import math
from sensor_msgs.msg import LaserScan

from limo_node import getCurrentState
from adjust_speed import rad2deg

DEBUG_TURN = False
ANGLE_RANGE = 30

def pidNaiveTurn(mylimo, LIMO_ID):
    # turns the limo based on the turning velocity of the limo in front
    # this is a naive implementation, may need tweaks to error correct if off course

    # Get steering angle 
    current_state_msg = getCurrentState(mylimo, LIMO_ID)
    stateValues = current_state_msg.split(";")
    steeringAngle = stateValues[-1]

    if DEBUG_TURN:
        print("SteeringAngle:", steeringAngle)

    return steeringAngle

def pidTurn(mylimo, LIMO_ID):
    # this function implements error correction for turning using lidar readings
    # implementation currently being written in adjust_speed.py scan_callback

    return 0

# test code
if __name__ == '__main__':
    #testState = "121" + ";" + "5" + ";" + "70"
    pidNaiveTurn("1", "1")