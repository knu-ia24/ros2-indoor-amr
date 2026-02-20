#!/usr/bin/env python3

import sys
import rclpy
from geometry_msgs.msg import Twist
import termios
import tty

msg = """
-------------------------------------------------
------------control keyboard manual--------------
-------------------------------------------------

              left    front   right

frontword       q       w       e

angular         a       s       d

backword        z       x       c

-------------------------------------------------
to increase velocity press '['
to decrease velocity press ']'"""

# Dictionary for movement bindings
moveBindings = {
    # Forward direction
    'w': (1, 0),   # Forward
    'e': (1, -1),  # Forward and right
    'q': (1, 1),  # Forward and left
    # Rotation
    'a': (0, 1),  # Turn left in place
    'd': (0, -1),   # Turn right in place
    # Backward direction
    'x': (-1, 0),  # Backward
    'c': (-1, 1),  # Backward and right
    'z': (-1, -1), # Backward and left
    's': (0, 0)    # Stop
}

# Dictionary for speed adjustments
speedBindings = {
    '[': (1.1, 1.1),  # Increase speed
    ']': (.9, .9)     # Decrease speed
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return 'currently:  linear_v = %s\tangular_v = %s' % ("{:.3f}".format(speed), "{:.3f}".format(turn))

def main():
    settings = saveTerminalSettings()
    rclpy.init()
    node = rclpy.create_node('move_beagle')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    vel = 0.12
    angular_vel = 0.8
    left = 0.0
    right = 0.0
    status = 0.0
    max_vel = 0.20
    max_angular_vel = 1.0

    try:
        print(msg)

        while True:
            key = getKey(settings)

            if key in moveBindings.keys():
                left = moveBindings[key][0]
                right = moveBindings[key][1]
                print(vels(left * vel, right * angular_vel))
            elif key in speedBindings.keys():
                vel = vel * speedBindings[key][0]
                angular_vel = angular_vel * speedBindings[key][1]

                # 속도 제한 로직 추가
                if vel > max_vel:
                    vel = max_vel
                if angular_vel > max_angular_vel:
                    angular_vel = max_angular_vel

                print(vels(vel, angular_vel))
                if status == 30:
                    print(msg)
                status = (status + 1) % 15
            else:
                if key == '\x03':
                    break
            twist = Twist()
            twist.linear.x = left * vel
            twist.angular.z = right * angular_vel
            pub.publish(twist)

    except Exception as r:
        print(r)
        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()
