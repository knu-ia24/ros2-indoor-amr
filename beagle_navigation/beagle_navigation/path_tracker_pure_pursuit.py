
import rclpy
from rclpy.node import Node
from nav_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
import tf_transformations
import numpy as np
import math
from std_msgs.msg import Bool



class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.global_path_sub_ = self.create_subscription(PoseStamped, '/goal_pose',self.goal_sub, 10)
        self.marker_pub = self.create_publisher(Marker, '/target_marker', 10)

        self.timer = self.create_timer(0.01,self.controller)

        self.path = Path()
        self.robot_pose = Odometry()
        self.LOOK_AHEAD_DISTANCE = 70.0
        self.x_values = []
        self.y_values = []
        self.goal = PoseStamped()
        self.path_calculating_sub = self.create_subscription(Bool, "/path_calculating", self.path_calculating_callback, 10)
        self.path_calculating = False  # 경로 계산 중 플래그




    def goal_sub(self, msg):

        self.goal.pose.position.x = msg.pose.position.x
        self.goal.pose.position.y = msg.pose.position.y
        
    def path_calculating_callback(self, msg):
        self.path_calculating = msg.data 

    def odom_callback(self, msg):

        self.robot_pose = msg

    def path_callback(self, msg):

        self.path = msg

    def get_next_target(self):
        min_distance = float('inf')
        target_point = None

        for pose in self.path.poses:
            distance = math.sqrt((pose.pose.position.x - self.robot_pose.pose.pose.position.x) ** 2 +
                                (pose.pose.position.y - self.robot_pose.pose.pose.position.y) ** 2)

            if distance < min_distance:
                min_distance = distance
                target_point = pose

        if target_point is None:
            return None

        path_length = len(self.path.poses)
        current_index = self.path.poses.index(target_point)
        look_ahead_index = min(current_index + int(self.LOOK_AHEAD_DISTANCE), path_length - 1)

        if look_ahead_index >= path_length - 1 or self.path.poses[look_ahead_index] is None:
            return None

        target_point = self.path.poses[look_ahead_index]

        return target_point

    def controller(self):

        if self.path_calculating:  
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.cmd_pub.publish(cmd_msg)
            return 

        target_point = self.get_next_target()
        if target_point is not None:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = target_point.pose.position.x - 0.075
            marker.pose.position.y = target_point.pose.position.y
            marker.scale.x = 0.15
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.marker_pub.publish(marker)
            
        


        if target_point is not None: # 목표점이 발견되는 경우

            # 로봇의 pose와 목표점간의 delta거리
            dx = target_point.pose.position.x - self.robot_pose.pose.pose.position.x
            dy = target_point.pose.position.y - self.robot_pose.pose.pose.position.y

            # in calculating self.yaw() there has problume between 179...degree to 180...degree
            # because it convert 179.. degree to 3.14... and 180... to -3.14...
            # so we need to convert one more time
            alpha = np.arctan2(dy, dx) - self.yaw(self.robot_pose.pose.pose.orientation)

            if alpha > math.pi:
                alpha -= 2 * math.pi
            elif alpha < -math.pi:
                alpha += 2 * math.pi

            # 조향각 계산
            steering_angle = 0.5 * math.atan(2 *  math.sin(alpha) / 0.1)

            # 로봇의 속력 설정
            linear_velocity = 0.08

            if np.absolute(alpha) > 0.25 :

                linear_velocity = 0.08 - np.absolute(alpha)/20.0

            cmd_msg = Twist()
            cmd_msg.linear.x = linear_velocity
            cmd_msg.angular.z = steering_angle


            if(math.sqrt((self.robot_pose.pose.pose.position.x - self.goal.pose.position.x)**2 + (self.robot_pose.pose.pose.position.y - self.goal.pose.position.y)**2) < 0.05):

                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0

                self.cmd_pub.publish(cmd_msg)
            else:
                self.cmd_pub.publish(cmd_msg)

    def yaw(self, orientation):
        _, _, yaw = tf_transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        return yaw



def main(args=None):
    rclpy.init(args=args)
    
    

    pure_pursuit_node = PurePursuitController()

    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
