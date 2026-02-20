#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import math
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial import cKDTree

from nav_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

show_animation = False

map_ = OccupancyGrid()

class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("calculate error! can't calculate this way")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            if show_animation:
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")

                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]

            closed_set[c_id] = current

            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):

        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index

        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 0.1
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):

        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):

        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False
        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = map_.info.origin.position.x / map_.info.resolution
        self.min_y = map_.info.origin.position.y / map_.info.resolution
        self.max_x = map_.info.width + map_.info.origin.position.x / map_.info.resolution
        self.max_y = map_.info.height + map_.info.origin.position.y / map_.info.resolution
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        obstacle_points = np.column_stack((ox, oy))
        kd_tree = cKDTree(obstacle_points)

        self.obstacle_map = np.zeros((self.x_width, self.y_width), dtype=bool)
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                nearest_index = kd_tree.query([x, y], k=1)[1]
                nearest_obstacle = obstacle_points[nearest_index]
                d = np.linalg.norm(np.array([x, y]) - nearest_obstacle)
                if d <= self.rr:
                    self.obstacle_map[ix, iy] = True

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, 1.2],
                  [-1, 1, 1.2],
                  [1, -1, 1.2],
                  [1, 1, 1.2]]

        return motion

class MyNode(Node):

    def __init__(self):
        super().__init__("subscribing_map_data")
        self.get_logger().info("sub open")

        self.subscriber_ = self.create_subscription(OccupancyGrid,"/map",self.subscriber_callback,10)
        self.gole_pose_subscriber_ = self.create_subscription(PoseStamped, "/goal_pose", self.astar_planner,10)
        self.pubscribe_odometry = self.create_subscription(Odometry,"/odom",self.odometry_callback,10)
        self.path_publisher_ = self.create_publisher(Path,"/global_path",10)

        self.ox = []
        self.oy = []
        self.i1 = []
        self.i2 = []

        self.path_msg = Path()
        self.cnt = 0

        self.odom_pose_x = 0.0
        self.odom_pose_y = 0.0
        self.path_calculating = False  
        self.path_calc_pub = self.create_publisher(Bool, "/path_calculating", 10)

    def smooth_path(self, cx, cy, smoothness):
        t = np.arange(0, len(cx), 1)
        fx = interp1d(t, cx, kind='cubic')
        fy = interp1d(t, cy, kind='cubic')

        t_smooth = np.arange(0, len(cx)-1, smoothness)
        cx_smooth = fx(t_smooth)
        cy_smooth = fy(t_smooth)

        return cx_smooth, cy_smooth

    def odometry_callback(self,msg):

        self.odom_pose_x = msg.pose.pose.position.x
        self.odom_pose_y = msg.pose.pose.position.y


    def subscriber_callback(self,msg):

        map_.info.width = msg.info.width
        map_.info.height = msg.info.height
        map_.info.resolution = msg.info.resolution
        map_.info.origin.position.x = msg.info.origin.position.x
        map_.info.origin.position.y = msg.info.origin.position.y
        map_.data = msg.data

        grid_map = np.array(map_.data).reshape(map_.info.height, map_.info.width)
        obstacle_indices = np.where((grid_map == -1) | (grid_map == 100))
        obstacle_positions_x = obstacle_indices[1] + map_.info.origin.position.x / map_.info.resolution
        obstacle_positions_y = obstacle_indices[0] + map_.info.origin.position.y / map_.info.resolution

        self.ox.extend(obstacle_positions_x)
        self.oy.extend(obstacle_positions_y)

    def astar_planner(self,msg):
        self.path_calc_pub.publish(Bool(data=True))

        goal = PoseStamped()
        goal.pose.position.x = msg.pose.position.x/map_.info.resolution
        goal.pose.position.y = msg.pose.position.y/map_.info.resolution

        self.odom_pose_x = self.odom_pose_x / map_.info.resolution
        self.odom_pose_y = self.odom_pose_y / map_.info.resolution

        if show_animation:
            plt.plot(self.ox, self.oy, ".k")
            plt.plot(self.odom_pose_x, self.odom_pose_y, "og")
            plt.plot(goal.pose.position.x, goal.pose.position.y, "xb")
            plt.grid(True)
            plt.axis("equal")


        grid_size = 0.2
        robot_radius =  0.08/map_.info.resolution

        print("!!!!!!!!!!!!!!!!!!!")
        a_star = AStarPlanner(self.ox, self.oy, grid_size, robot_radius)
        #print("Initializing A* Planner with map resolution:", map_.info.resolution)
        #print("Initializing A* Planner with rr:", robot_radius)
        print("!!!!!!!!!!!!!!!!!!!!")

        cx_path, cy_path = a_star.planning(self.odom_pose_x,self.odom_pose_y,goal.pose.position.x,goal.pose.position.y)

        if show_animation:
            plt.plot(cx_path, cy_path, "-r")
            plt.pause(0.001)
            plt.show()
            
        print("!!!!!!!!!!!")

        if len(cx_path) < 3 or len(cy_path) < 3:
            #print(len(cx_path))
            #print(len(cy_path))
            self.get_logger().warn("Not enough points for path smoothing")
            return

        cx_path = np.array(cx_path[::-1])
        cy_path = np.array(cy_path[::-1])
        cx_path,cy_path = self.smooth_path(cx_path,cy_path,0.1)#smoothness

        self.path_msg.header.frame_id = "/map"
        for i in range (0,len(cx_path)):
            pose = PoseStamped()
            pose.pose.position.x = cx_path[i]*map_.info.resolution
            pose.pose.position.y = cy_path[i]*map_.info.resolution
            self.path_msg.poses.append(pose)

        self.path_publisher_.publish(self.path_msg)
        self.path_msg = Path()
        #print(len(cx_path))
        #print(len(cy_path))
        self.path_calc_pub.publish(Bool(data=False))

def main(args = None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()
