#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan

counter = 0

def RAD2DEG(r):
  return r * 180 / math.pi


def callback(event):
  global counter
  counter += 1
  rospy.loginfo("sample file called : %d times", counter)
    
    
def scanCallback(scan):
  scan_num = int(round(( (scan.angle_max - scan.angle_min) / scan.angle_increment ), -1))
  for i in range(scan_num):
    degree = RAD2DEG(scan.angle_increment * i)	# The first point is defined as 0 degrees.
    print("[LIDAR INFO]:angle-distance:[{:>4.1f}, {:>5.3f}]".format(degree, scan.ranges[i]))


def listener():
  rospy.init_node('py_sample')
  
  timer1 = rospy.Timer(rospy.Duration(0.1), callback)
  
  sub = rospy.Subscriber("/scan", LaserScan, scanCallback)
  
  rospy.spin()

if __name__ == '__main__':
  listener()
