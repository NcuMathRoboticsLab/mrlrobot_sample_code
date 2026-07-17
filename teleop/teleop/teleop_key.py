#!/usr/bin/env python3
# coding=utf-8
import os
import select
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

if os.name == 'nt':
    import msvcrt
    import time
else:
    import tty
    import termios

TURTLEBOT_MAX_LIN_VEL = 0.22
TURTLEBOT_MAX_ANG_VEL = 2.84

TURTLEBOT_LIN_VEL_STEP_SIZE = 0.01
TURTLEBOT_ANG_VEL_STEP_SIZE = 0.1

MINIBOT_MAX_LIN_VEL = 0.28
MINIBOT_MAX_ANG_VEL = 2.8

MINIBOT_LIN_VEL_STEP_SIZE = 0.04
MINIBOT_ANG_VEL_STEP_SIZE = 0.35


msg = '''
Control Your Robot
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (turtlebot : ~ 0.22, minibot : ~ 0.2)
a/d : increase/decrease angular velocity (turtlebot : ~ 2.84, minibot : ~ 1.0)

space key, s : force stop

CTRL-C to quit
'''

e = '''
Communications Failed
'''


def getKey(settings):
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while 1:
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(target_linear_vel, target_angular_vel):
    return 'currently:\tlinear vel %s\t angular vel %s ' % (
        target_linear_vel,
        target_angular_vel,
    )


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input


def checkLinearLimitVelocity(mrl_robot_model, vel):
    if mrl_robot_model == 'turtlebot':
        vel = constrain(vel, -TURTLEBOT_MAX_LIN_VEL, TURTLEBOT_MAX_LIN_VEL)
    elif mrl_robot_model == 'minibot':
        vel = constrain(vel, -MINIBOT_MAX_LIN_VEL, MINIBOT_MAX_LIN_VEL)
    else:
        vel = constrain(vel, -TURTLEBOT_MAX_LIN_VEL, TURTLEBOT_MAX_LIN_VEL)
    return vel


def checkAngularLimitVelocity(mrl_robot_model, vel):
    if mrl_robot_model == 'turtlebot':
        vel = constrain(vel, -TURTLEBOT_MAX_ANG_VEL, TURTLEBOT_MAX_ANG_VEL)
    elif mrl_robot_model == 'minibot':
        vel = constrain(vel, -MINIBOT_MAX_ANG_VEL, MINIBOT_MAX_ANG_VEL)
    else:
        vel = constrain(vel, -TURTLEBOT_MAX_ANG_VEL, TURTLEBOT_MAX_ANG_VEL)
    return vel


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')

        self.declare_parameter('model', os.getenv('MRL_ROBOT_MODEL', 'minibot'))

        self.model = self.get_parameter('model').value

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)


def main(args=None):
    rclpy.init(args=args)

    node = TeleopKey()

    robot_model = node.model

    settings = termios.tcgetattr(sys.stdin)

    if robot_model == 'turtlebot':
        lin_vel_step_size = TURTLEBOT_LIN_VEL_STEP_SIZE
        ang_vel_step_size = TURTLEBOT_ANG_VEL_STEP_SIZE
        print('You are using turtlebot.')
    elif robot_model == 'minibot':
        lin_vel_step_size = MINIBOT_LIN_VEL_STEP_SIZE
        ang_vel_step_size = MINIBOT_ANG_VEL_STEP_SIZE
        print('You are using minibot.')
    else:
        lin_vel_step_size = TURTLEBOT_LIN_VEL_STEP_SIZE
        ang_vel_step_size = TURTLEBOT_ANG_VEL_STEP_SIZE

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while rclpy.ok():
            key = getKey(settings)
            if key == 'w':
                target_linear_vel = checkLinearLimitVelocity(
                    robot_model, target_linear_vel + lin_vel_step_size
                )
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'x':
                target_linear_vel = checkLinearLimitVelocity(
                    robot_model, target_linear_vel - lin_vel_step_size
                )
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'a':
                target_angular_vel = checkAngularLimitVelocity(
                    robot_model, target_angular_vel + ang_vel_step_size
                )
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'd':
                target_angular_vel = checkAngularLimitVelocity(
                    robot_model, target_angular_vel - ang_vel_step_size
                )
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == ' ' or key == 's':
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if key == '\x03':
                    break

            if status == 20:
                print(msg)
                status = 0

            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = node.get_clock().now().to_msg()
            twist_stamped.header.frame_id = ''

            control_linear_vel = makeSimpleProfile(
                control_linear_vel, target_linear_vel, (lin_vel_step_size / 2.0)
            )
            twist_stamped.twist.linear.x = control_linear_vel
            twist_stamped.twist.linear.y = 0.0
            twist_stamped.twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(
                control_angular_vel, target_angular_vel, (ang_vel_step_size / 2.0)
            )
            twist_stamped.twist.angular.x = 0.0
            twist_stamped.twist.angular.y = 0.0
            twist_stamped.twist.angular.z = control_angular_vel

            node.pub.publish(twist_stamped)

    except Exception:
        print(e)

    finally:
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = node.get_clock().now().to_msg()
        twist_stamped.header.frame_id = ''
        node.pub.publish(twist_stamped)  # stop the robot

        node.destroy_node()
        rclpy.shutdown()

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
