import math
from sensor_msgs.msg import LaserScan

DEBUG_LIDAR = False
ANGLE_RANGE = 6

SETPT = 0.4
MIN_DIST = SETPT - 0.02
MAX_DIST = SETPT + 0.02

Kp = 0.7 # Proportional constant
Ki = 0.04 # Integral constant
Kd = 0.30 # Derivative constant

distance = 0

def rad2deg(x):
    return (x * 180.0) / math.pi

# Get lidar data
def scan_callback(scan):
    count = math.floor(scan.scan_time / scan.time_increment)
    distances = []
    
    for i in range(count):
        degree = rad2deg(scan.angle_min + scan.angle_increment * i)

        # Only take LiDAR data in front of limo
        if degree >= (-1 * ANGLE_RANGE) and degree < ANGLE_RANGE:
            dist = scan.ranges[i]
            if dist > 0:
                distances.append(dist)
            if DEBUG_LIDAR:
                print(degree, dist, i)
    
    # Average the data
    mean = 0
    if len(distances) > 0:
        mean = sum(distances) / len(distances)
    global lastNZdist
    global distance
    #distance = mean
    if mean>0:
        distance = mean
        lastNZdist = mean
    else:
        distance = lastNZdist

def pid(rate_hz, prevError, prevIntegral):
    if distance < MIN_DIST: 
        error = MIN_DIST - distance
    elif distance > MAX_DIST:
        error =  MAX_DIST - distance
    else:
        error = 0
    # PID algorithm
    proportional = error
    integral =min(prevIntegral + error,1)
    derivative = error - prevError
    output = Kp * proportional + Ki * integral + Kd * derivative

    '''
    Expected "output" value:
        * 0 : no error
        * + : front limo is too close
        * - : front limo is too far
    '''
    adjustSpeed = 0
    if output != 0:
        adjustSpeed = -1 * output

    print("[Distance, Adjustment]: ", distance,adjustSpeed)

    return adjustSpeed, error, integral

    
            

        
        


                
