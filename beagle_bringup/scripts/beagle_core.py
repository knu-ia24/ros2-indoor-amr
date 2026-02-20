#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from roboid import *
import numpy as np
import time
from math import *
import tf_transformations
from tf2_ros import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import *
from nav_msgs.msg import *

class MyNode(Node):

    def __init__(self):
        super().__init__("beagle_core")
        self.create_timer(0.01, self.timer_callback)
        self.publisher_ = self.create_publisher(LaserScan,"/scan", 10000)
        self.publisher_odom = self.create_publisher(Odometry,"/odom", 10000)
        self.beagle_path_publisher = self.create_publisher(Path,"/moved_path",10000)
        self.subscriber_ = self.create_subscription(Twist,"/cmd_vel",self.subscriber_callback,10000)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.beagle = Beagle()

        self.value = [0.0,]
        self.nvalue = [0.0,]

        self.last_cmd_RPS_L = 0.0
        self.last_cmd_RPS_R = 0.0

        self.last_cmd_vel_L = 0.0
        self.last_cmd_vel_R = 0.0

        self.beagle_vel_L = 0.0
        self.beagle_vel_R = 0.0

        self.left_vel = 0.0
        self.right_vel = 0.0

        self.recent_x = 0.0
        self.recent_y = 0.0
        self.recent_theta = 0.0000000000000

        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.00000000

        self.wheel_radius = 0.033

        self.recent_time = time.time()
        self.last_time = time.time()

        self.dt = 0.0
        self.wheel_base = 0.09560
        self.max_rpm = 93.75

        self.quaternion = [0.0,0.0,0.0,0.0]

        self.odom = Odometry()

        self.timer_cnt = 0

        self.ros_time = 0.0

        self.beagle_moved_path = Path()
        self.beagle_moved_path.header.frame_id = "/odom"

    def timer_callback(self):

        self.recent_time = time.time()
        self.dt = self.recent_time - self.last_time

        self.beagle.write(self.beagle.LEFT_WHEEL, self.beagle_vel_L)
        self.beagle.write(self.beagle.RIGHT_WHEEL, self.beagle_vel_R)

        if self.timer_cnt % 20 == 0 :
            self.ros_time = self.get_clock().now()
            self.publish_odom()
            self.publish_lidar()
            self.tf_publisher()
            
            self.timer_cnt = 0
            self.beagle_path_publisher.publish(self.beagle_moved_path)
        self.update_pose()

        self.last_time = self.recent_time
        self.timer_cnt += 1

    def subscriber_callback(self,msg):

        self.last_cmd_vel_L = msg.linear.x
        self.last_cmd_vel_R = msg.angular.z

        self.left_vel = msg.linear.x - (msg.angular.z * self.wheel_base / 2)
        self.right_vel = msg.linear.x + (msg.angular.z * self.wheel_base / 2)

        self.last_cmd_RPS_L = self.left_vel / self.wheel_radius
        self.last_cmd_RPS_R = self.right_vel / self.wheel_radius

        self.beagle_vel_L = self.last_cmd_RPS_L/ (self.max_rpm / 60.0 * 2 * pi) * 100.0
        self.beagle_vel_R = self.last_cmd_RPS_R/ (self.max_rpm / 60.0 * 2 * pi) * 100.0

        self.odom.twist.twist.linear.x = msg.linear.x
        self.odom.twist.twist.angular.z = msg.angular.z


    def publish_lidar(self):
        self.value = self.beagle.lidar()
        msg = LaserScan()

        current_time = self.ros_time
        
        msg.header.frame_id = 'laser'
        msg.angle_min = 0.0
        msg.angle_max = pi*2
        msg.angle_increment = pi/180
        msg.time_increment = 0.001
        msg.scan_time = 0.2
        msg.range_max = 8.0   #black 8.0 white 12.0
        msg.range_min = 0.05
        msg.intensities = [0.0] * 360

        tmp = np.zeros((360), dtype=np.float32)

        for i in range(0,360):
            if self.value[i] >= 0 and self.value[i] <= 5000 :
                tmp[i] = self.value[i]/1000.0
            else :
                tmp[i] = np.Inf

        msg.ranges = tmp.tolist()
        msg.header.stamp = current_time.to_msg()
        self.publisher_.publish(msg)


    def update_pose(self):

        self.recent_x = self.last_x + self.dt*(self.left_vel + self.right_vel)*cos(self.last_theta)/2.0
        self.recent_y = self.last_y + self.dt*(self.left_vel + self.right_vel)*sin(self.last_theta)/2.0
        if (self.beagle.gyroscope_z() > 1.0 or self.beagle.gyroscope_z() < -1.0):
            self.recent_theta = self.last_theta + self.dt*self.beagle.gyroscope_z()*pi/180.0000000
            print(self.beagle.gyroscope_z() , self.recent_theta)

        self.last_x = self.recent_x
        self.last_y = self.recent_y
        self.last_theta = self.recent_theta



        beagle_pose = PoseStamped()
        beagle_pose.pose.position.x = self.last_x
        beagle_pose.pose.position.y = self.last_y

        self.beagle_moved_path.poses.append(beagle_pose)

    def publish_odom(self):
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'laser'
        self.odom.pose.pose.position.x = self.recent_x
        self.odom.pose.pose.position.y = self.recent_y

        self.quaternion = tf_transformations.quaternion_from_euler(0,0,self.recent_theta)
        self.odom.pose.pose.orientation.x = self.quaternion[0]
        self.odom.pose.pose.orientation.y = self.quaternion[1]
        self.odom.pose.pose.orientation.z = self.quaternion[2]
        self.odom.pose.pose.orientation.w = self.quaternion[3]

        self.odom.header.stamp = self.ros_time.to_msg()

        self.publisher_odom.publish(self.odom)

    def tf_publisher(self):

        t = TransformStamped()

        t.header.stamp = self.ros_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'laser'

        t.transform.translation.x = self.odom.pose.pose.position.x
        t.transform.translation.y = self.odom.pose.pose.position.y

        t.transform.rotation.x = self.odom.pose.pose.orientation.x
        t.transform.rotation.y = self.odom.pose.pose.orientation.y
        t.transform.rotation.z = self.odom.pose.pose.orientation.z
        t.transform.rotation.w = self.odom.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

def main(args = None):

    rclpy.init(args=args)
    node = MyNode()
    node.beagle.start_lidar()
    node.beagle.wait_until_lidar_ready()
    print('lidar is ready.')

    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()
