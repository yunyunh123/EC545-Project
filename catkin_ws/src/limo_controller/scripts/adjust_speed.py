import math
from sensor_msgs.msg import LaserScan

DEBUG_LIDAR = False
ANGLE_RANGE = 6

LEFT_SENSOR_VAL = -.85956 # left
RIGHT_SENSOR_VAL = 0.5576 # right

TURN_ANGLE_RANGE = 35 # degrees
TURN_CLOSEST_PERCENT = 10 # percent

SETPT = 0.4
MIN_DIST = SETPT - 0.15
MAX_DIST = SETPT + 0.15

Kp = 0.7 # Proportional constant
Ki = 0.04 # Integral constant
Kd = 0.30 # Derivative constant

distance = 0
steeringAngle = 0
turnDistances = []

def rad2deg(x):
    return (x * 180.0) / math.pi

# Get lidar data
def scan_callback(scan):
    count = math.floor(scan.scan_time / scan.time_increment)
    distances = []
    global turnDistances
    
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
                turnDistances.append(dist)

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

    print("Distance: ", distance)
    
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

    steeringAngle = adjustAngle
    if steeringAngle == None:
        steeringAngle = 0

    return steeringAngle, adjustSpeed, error, integral

def adjustAngle():
    # ----- Turning implementation 
    # Declare variables for the implementation
    global turnDistances
    steeringMatrix = [0] * len(turnDistances) # array that holds steering angle with same indexes as the turn distances
    
    # Calculate the distances for the right and left sides of the data set
    try:
        numHalfTurnAngle = int(len(turnDistances) / 2)
        distBetweenMeasurementsLeft = LEFT_SENSOR_VAL / numHalfTurnAngle
        distBetweenMeasurementsRight = RIGHT_SENSOR_VAL / numHalfTurnAngle
    except ZeroDivisionError:
        numHalfTurnAngle = 0 
        distBetweenMeasurementsLeft = 0
        distBetweenMeasurementsRight = 0
    
    
    # Calculate the steering angle towards each datapoint and insert into an array
    turnAngle = LEFT_SENSOR_VAL
    for index, dataPoint in enumerate(turnDistances):
        steeringMatrix[index] = turnAngle

        if index < numHalfTurnAngle: # handle the left side
            turnAngle = turnAngle - distBetweenMeasurementsLeft
        elif index > numHalfTurnAngle: # handle the right side
            turnAngle = turnAngle + distBetweenMeasurementsRight
        elif index == numHalfTurnAngle: # handle the center
            turnAngle = 0

    # Calculate the what datapoints are the closest
    numCloseValues = int((TURN_CLOSEST_PERCENT/100) * len(turnDistances)) # number of values in the top * percent
    sortedDistances = sorted(turnDistances)
    closestDistances = sortedDistances[:numCloseValues]

    indexArr = [turnDistances.index(value) for value in closestDistances] # the index values of the closest values
    print("Close values: ", closestDistances)

    if len(closestDistances) == 0:
        print("The array is empty")
    else:
        print("The array is not empty")

    # Average the steering angle towards these datapoints to get the needed steering angle
    closestAngles = []
    for index in indexArr:
        closestAngles.append(steeringMatrix[index])

    try:
        adjustedAngle = sum(closestAngles)/len(closestAngles)
        return adjustedAngle
    except ZeroDivisionError:
        adjustedAngle = adjustedAngle