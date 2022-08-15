#! /usr/bin/env python3

import rospy
import random
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def readLaser(msg):
    front = min(min(min(msg.ranges[0:45]), min(msg.ranges[315:360])),10)
    drive(front)

def drive(range):
    vel = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    if (range > .3):
        vel.linear.x = 0.2
        vel.angular.z = 0.0
    else:
        angle = (random.randint(0,360) * math.pi) / 180
        turn = random.randint(0,1)
        vel.linear.x = 0.0
        vel.angular.z = angle
    pub.publish(vel)

def main():

    rospy.init_node('explore')
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        read_Laser = rospy.Subscriber('/scan', LaserScan, readLaser) 
        rate.sleep()
        
        
if __name__ == "__main__":
    main()
