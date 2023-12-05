import math
from sensor_msgs.msg import LaserScan

DEBUG_LIDAR = False
ANGLE_RANGE = 6
TURN_ANGLE_RANGE = 30 # degrees
TURN_CLOSEST_PERCENT = 10 # percent

WHEELBASE = 8 # inches

SETPT = 0.3
MIN_DIST = 0.15
MAX_DIST = 0.45

Kp = 0.7 # Proportional constant
Ki = 0 # Integral constant
Kd = 1.5 # Derivative constant

distance = 0
steeringAngle = 0

def rad2deg(x):
    return (x * 180.0) / math.pi

def ackermannSteeringAngle(turn_radius):
    # NOT CORRECT

    # Calculate Ackermann steering angle in radians
    delta_radians = math.atan(WHEELBASE / turn_radius)

    # Convert radians to degrees
    delta_degrees = math.degrees(delta_radians)

    return delta_degrees

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
                turnDistances.append(dist)

            if DEBUG_LIDAR:
                print(degree, dist, i)
    
    # Average the data
    mean = 0
    if len(distances) > 0:
        mean = sum(distances) / len(distances)

    global distance
    distance = mean

    # ----- Turning implementation 
    # Declare variables for the implementation
    steeringMatrix = np.empty(len(turnDistances)) # array that holds steering angle with same indexes as the turn distances

    # Calculate the degree each datapoint is relative
    totalDegreesCovered = TURN_ANGLE_RANGE * 2
    distBetweenMeasurements = totalDegreesCovered / (len(turnDistances) - 1)
    
    # Calculate the steering angle towards each datapoint and insert into an array

    turnAngle = TURN_ANGLE_RANGE * -1
    for index, dataPoint in enumerate(turnDistances):
        """
        Caculation here.
        """
        
        # REPLACE WITH CALCULATED EQUATION RESULT - Currently saves degrees of each value
        steeringMatrix[index] = turnAngle 
        turnAngle = turnAngle + distBetweenMeasurements

    # Calculate the what datapoints are the closest (ex. 90% closest datapoints - FINE TUNE PERCENTAGE)
    numCloseValues = int((TURN_CLOSEST_PERCENT/100) * len(turnDistances)) # number of values in the top * percent
    sortedDistances = sorted(turnDistances)
    closestDistances = sortedDistances[:numCloseValues]

    indexArr = [turnDistances.index(value) for value in closestDistances] # the index values of the closest values

    # Average the steering angle towards these datapoints to get the needed steering angle
    closestAngles = []
    for index in indexArr:
        closestAngles.append(steeringMatrix[index])

    global steeringAngle
    steeringAngle = sum(closestAngles)/len(closestAngles)
    
    


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

    print(distance, output, adjustSpeed)

    return adjustSpeed, error, integral