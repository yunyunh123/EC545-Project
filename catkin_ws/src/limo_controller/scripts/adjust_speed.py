import math
from sensor_msgs.msg import LaserScan

DEBUG_LIDAR = False
ANGLE_RANGE = 6

SETPT = 0.4

MIN_DIST = SETPT - 0.02
MAX_DIST = SETPT + 0.02

INTEGRAL_MAX=0.5 #Adjusts max and min values for integral response


Kp = 0.7 # Proportional constant
Ki = 0.04 # Integral constant
Kd = 0.30 # Derivative constant

MIN_ANGLE = -0.574 # right
MAX_ANGLE = 0.85956 # left
FRONT_ANGLE_RANGE = 20

MEAN_COUNT = 5

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
            if dist > 0:
                distances.append(dist)
            
        if degree <= 90 and degree >= 60:
            test = scan.ranges[i]
            #if test>0:
                #print("degree, test: ", degree, test)
        #print(degree, scan.ranges[i])
        
        '''
        # Only take LiDAR data in front of limo
        if degree >= (-1 * ANGLE_RANGE) and degree < ANGLE_RANGE:
            dist = scan.ranges[i]
            if dist > 0:
                distances.append(dist)
        '''

    # Average the data
    distances = sorted(distances)[:MEAN_COUNT]
    #print(distances)
    mean = 0
    if len(distances) > 0:
        mean = sum(distances) / len(distances)
    #print(mean)
    global lastNZdist
    global distance
    
    if mean>0:
        distance = mean
        lastNZdist = mean
    else:
        distance = lastNZdist


    if closest_distance != float('inf'):
        closest_distances.append(closest_distance)
        closest_degrees.append(closest_degree)
        #print("[closest distance, closest degree]: ", closest_distance, closest_degree)

def pid(rate_hz, prevError, prevIntegral):
    global distance
    if distance < MIN_DIST: 
        error = MIN_DIST - distance
    elif distance > MAX_DIST:
        error =  MAX_DIST - distance
    else:
        error = 0
    # PID algorithm
    #proportional = max(min(error,2),-2)
    proportional = error
    derivative = error-prevError
    integral = max(min(prevIntegral + error,INTEGRAL_MAX),-INTEGRAL_MAX)
    #derivative = max(min(error - prevError,2),-2)
    output = Kp * proportional + Ki * integral + Kd * derivative
    output = output*1.2
    #print("[P, I, D]", proportional, integral, derivative)
    '''
    Expected "output" value:
        * 0 : no error
        * + : front limo is too close
        * - : front limo is too far
    '''
    adjustSpeed = 0
    if output != 0:
        adjustSpeed = -1 * output

    #print("[Distance, Adjustment]: ", distance,adjustSpeed)

    return adjustSpeed, error, integral

def adjust_angle():
    if len(closest_distances) == 0:
        return None

    newAngle = 0
    while len(closest_distances) > MEAN_COUNT:
        closest_distances.pop(0)
        closest_degrees.pop(0)

    ave_closest_dist = 0
    ave_closest_degree = 0
    div = 0
    for i in range(len(closest_distances)):
        div += i + 1
        ave_closest_dist += closest_distances[i] * (i+1)
        ave_closest_degree += closest_degrees[i] * (i+1)
    ave_closest_dist = ave_closest_dist / div
    ave_closest_degree = ave_closest_degree / div 

    # set degree to 0 if its within "front" range
    if ave_closest_degree < 15 and ave_closest_degree > -15:
        ave_closest_degree = 0.001    
    

    newAngle = ave_closest_degree / 60.0
    #print("[angle, degree, dist]: ", newAngle, ave_closest_degree, ave_closest_dist)


    global distance
    print("[degree, dist]", ave_closest_degree, distance)

    return float(newAngle)

