import math
from sensor_msgs.msg import LaserScan

DEBUG_LIDAR = True
ANGLE_RANGE = 6

def rad2deg(x):
    return (x * 180.0) / math.pi

def scan_callback(scan):
    print("here")
    count = math.floor(scan.scan_time / scan.time_increment)
    distances = []
    
    for i in range(count):
        degree = rad2deg(scan.angle_min + scan.angle_increment * i)
        if degree >= (-1 * ANGLE_RANGE) and degree < ANGLE_RANGE:
            dist = scan.ranges[i]
            if dist > 0:
                distances.append(dist)
            if DEBUG_LIDAR:
                print(degree, dist, i)

    #mean = 0
    #if len(distances) > 0:
    #    mean = sum(distances) / len(distances)
    #print(mean)
    
            

        
        


                
