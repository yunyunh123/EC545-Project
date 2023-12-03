import rospy 
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from pylimo import limo
import math

def rad2deg(x):
    return (x * 180.0) / math.pi

def scanCallback(scan):
    count = math.floor(scan.scan_time / scan.time_increment)
    for i in range(count):
        degree = rad2deg(scan.angle_min + scan.angle_increment * i)
        if degree > -5 and degree < 5:
            print(degree, scan.ranges[i], i)



if __name__ == '__main__':
        rospy.init_node("get_lidar")
        rospy.loginfo("Created get_lidar node")
        sub = rospy.Subscriber("/scan", LaserScan, scanCallback)
        rospy.spin()
