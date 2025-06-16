#!/usr/bin/env python3
# coding=utf-8
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from math import pi
from sensor_msgs.msg import LaserScan

def RAD2DEG(r):
  return r * 180 / pi

class PySample(Node):
  def __init__(self):
    super().__init__('py_sample')
    self.counter = 0

    # 建立 Timer，0.1秒
    self.create_timer(0.1, self.timer_callback)

    # 建立 Subscriber，以 Best Effort 訂閱 /scan
    self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

  def timer_callback(self):
    self.counter += 1
    self.get_logger().info(f'sample file called : {self.counter} times')

  def scan_callback(self, scan):
    scan_num = int(round(( (scan.angle_max - scan.angle_min) / scan.angle_increment ), -1))
    for i in range(scan_num):
      degree = RAD2DEG(scan.angle_increment * i)	# The first point is defined as 0 degrees.
      print(f'[LIDAR INFO]:angle-distance:[{degree:>4.1f}, {scan.ranges[i]:>5.3f}]')


def main(args=None):
  rclpy.init(args=args)

  node = PySample()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    print('KeyboardInterrupt')
  finally:
    node.destroy_node()

if __name__ == '__main__':
  main()