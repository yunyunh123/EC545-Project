import math
from sensor_msgs.msg import LaserScan

DEBUG_LIDAR = False
ANGLE_RANGE = 6

SETPT = 0.4
MIN_DIST = SETPT - 0.15
MAX_DIST = SETPT + 0.15

Kp = 0.7 # Proportional constant
Ki = 0.04 # Integral constant
Kd = 0.30 # Derivative constant

MIN_ANGLE = -0.574 # right
MAX_ANGLE = 0.85956 # left
FRONT_ANGLE_RANGE = 20

distance = 0
closest_distances = []
closest_degrees = []
closest_distance = 0
closest_degree = 0

def rad2deg(x):
    return (x * 180.0) / math.pi

# Get lidar data
def scan_callback(scan):
    count = math.floor(scan.scan_time / scan.time_increment)
    distances = []
    closest_distance = float('inf')
    closest_degree = 0
    for i in range(count):
        degree = rad2deg(scan.angle_min + scan.angle_increment * i)

        # Find closest object
        if i < len(scan.ranges):
            dist = scan.ranges[i]
            if dist > 0 and dist < closest_distance:
                closest_distance = dist
                closest_degree = degree

        #print(degree, scan.ranges[i])


        # Only take LiDAR data in front of limo
        if degree >= (-1 * ANGLE_RANGE) and degree < ANGLE_RANGE:
            dist = scan.ranges[i]
            if dist > 0:
                distances.append(dist)
            if DEBUG_LIDAR:
                print(degree, dist, i)
    
    # Average the data
    ''' Maybe move down to update_speed()?
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
    '''

    if closest_distance != float('inf'):
        closest_distances.append(closest_distance)
        closest_degrees.append(closest_degree)
        #print("[closest distance, closest degree]: ", closest_distance, closest_degree)

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

    print("[Distance, Adjustment]: ", distance,adjustSpeed)

    return adjustSpeed, error, integral

def adjust_angle():
    if len(closest_distances) == 0:
        return None

    newAngle = 0
    while len(closest_distances) > 7:
        closest_distances.pop(0)
        closest_degrees.pop(0)

    ave_closest_dist = sum(closest_distances)/len(closest_distances)
    ave_closest_degree = sum(closest_degrees)/len(closest_degrees)
    global distance
    distance = ave_closest_dist

    # potentially set degree to 0 if its within "front" range

    newAngle = ave_closest_degree / 60.0
    #print("[angle, degree, dist]: ", newAngle, ave_closest_degree, ave_closest_dist)
    return float(newAngle)

