import math
from sensor_msgs.msg import LaserScan

DEBUG_LIDAR = False
ANGLE_RANGE = 6

ANGLE_ESTIMATE = 60
LEFT_SENSOR_VAL = .85956 # left
RIGHT_SENSOR_VAL = -0.5576 # right

TURN_ANGLE_RANGE = 40 # degrees
TURN_CLOSEST_PERCENT = 10 # percent
TURN_ERROR_TOLERANCE = .1
TURN_DISTANCE_SIZE = TURN_ANGLE_RANGE * 9 # three datapoints per degree, take this measurement thrice

SETPT = 0.4
MIN_DIST = SETPT - 0.15
MAX_DIST = SETPT + 0.15

Kp = 0.7 # Proportional constant
Ki = 0.04 # Integral constant
Kd = 0.30 # Derivative constant

distance = 0
steeringAngle = 0

def rad2deg(x):
    return (x * 180.0) / math.pi

# Get lidar data
def scan_callback(scan):
    count = math.floor(scan.scan_time / scan.time_increment)
    distances = []

    turnDistances = []
    
    for i in range(count):
        degree = rad2deg(scan.angle_min + scan.angle_increment * i)

        # Only take LiDAR data in front of limo
        if degree >= (-1 * ANGLE_RANGE) and degree < ANGLE_RANGE:
            dist = scan.ranges[i]
            if dist > 0:
                distances.append(dist)
            if DEBUG_LIDAR:
                print(degree, dist, i)

        # Get LiDAR data from wider range
        if degree >= (-1 * TURN_ANGLE_RANGE) and degree < TURN_ANGLE_RANGE:
            dist = scan.ranges[i]
            if dist > 0:
                turnDistances.append((dist, degree))
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

    # ----- Turning implementation     
    if len(turnDistances) > 10: # if we have a sufficient sample
        print("Distances: ", turnDistances, "\n")
        # Calculate the what datapoints are the closest
        numCloseValues = int((TURN_CLOSEST_PERCENT/100) * len(turnDistances)) # number of values in the top x percent
        sortedDistances = sorted(turnDistances, key=lambda x: x[0])
        closestDistances = sortedDistances[:numCloseValues]

    # Get the scan angles from the closest distances
        closestAngles = []
        for dist, degree in closestDistances:
            closestAngles.append(degree)

        # Average the angles and convert to a steering angle
        global steeringAngle
        try:
            averageAngle = sum(closestAngles)/len(closestAngles)
            potentialAngle = averageAngle / 60.0

            if averageAngle > 0:
                steeringAngle = potentialAngle * 1.1 # to account for greater max value on this end
            elif averageAngle < 0:
                steeringAngle = potentialAngle
        except ZeroDivisionError:
            next
        print("Steering angle: ", steeringAngle)
    else:
        next # skip and wait for more data

def pid(rate_hz, prevError, prevIntegral):

    error = SETPT - distance

    # PID algorithm
    proportional = error
    integral = prevIntegral + error
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
    
    global steeringAngle
    return steeringAngle, adjustSpeed, error, integral

    
            

        
        


                
