#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from tf_transformations import euler_from_quaternion
import numpy as np
import math
###############################################################################
node_name = "bug_control"

class MyNode(Node):

    def __init__(self):
        super().__init__(node_name)
        self.get_logger().info(f"Starting Node : {node_name}")

        #####################Subscribers & Publishers#######################################
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, QoSProfile(reliability = QoSReliabilityPolicy.BEST_EFFORT, depth = 10))

        ####################################################################################
        # Initialize Variables
        self.initial_x = None
        self.initial_y = None
        self.initial_z = None
        self.target_set = False  # Ensure target is set only once
        #Target variables
        self.target_point = Point()
        self.target_point.x = 0.0
        self.target_point.y = 0.0
        self.target_point.z = 0.0
        #Delta Variables
        self.delta_x = None
        self.delta_y = None
        self.delta_z = None
        self.target_orinet = None

        self.delta_distance = None 
        #Control Variables
        self.velocity = 0.0
        self.omega = 0.0
        self.yaw = 0.0
        
        self.left_obsta = False
        self.right_obsta = False

        self.front_distance = None
        self.right_stop = []
        self.left_stop = []

        self.state = "MOVE_TARGET"
        self.backup = ""
        self.move_forward = 0
        self.rotate = 0
        
        ####################################################################################
        # Timer for publishing target marker
        self.create_timer(1.0, self.publish_target_marker)
        self.create_timer(0.5,self.control)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        if not self.target_set:
            self.initial_x = 0.0
            self.initial_y = 0.0
            self.initial_z = 0.0
            self.target_point.x = 4.0
            self.target_point.y = 0.0
            self.target_point.z = 0.0

            self.target_set = True  
            self.get_logger().info(f"Target at X,Y,Z: {self.target_point.x:.5f}, {self.target_point.y:.5f}, {self.target_point.z:.5f}")
        
        self.delta_x = self.target_point.x - position.x
        self.delta_y = self.target_point.y - position.y
        self.delta_z = self.target_point.z - position.z
        self.delta_distance = np.sqrt(self.delta_x**2 + self.delta_y**2 + self.delta_z**2)
        self.target_orinet = math.atan2(self.delta_y,self.delta_x)

        lidar_value = self.front_distance if self.front_distance is not None else float('inf')

        #self.get_logger().info(f"Turtlebot: {position.x:.4f},{position.y:.4f},{position.z:.4f} | Target : {self.target_point.x:.4f}, {self.target_point.y:.4f}, {self.target_point.z:.4f} | Delta point : {self.delta_x:.4f},{self.delta_y:.4f},{self.delta_z:.4f} | Distance : {self.delta_distance} | Lidar : {lidar_value:.4f}")
        
        self.publish_line_marker(position)

    def publish_target_marker(self):
        if not self.target_set:
            return  

        marker = Marker()
        marker.header.frame_id = "odom"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = self.target_point
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)
        #self.get_logger().info(f"Published Target Marker at Positon: {self.target_point.x},{self.target_point.y},{self.target_point.z}")

    def publish_line_marker(self, robot_position):
        if not self.target_set:
            return
        
        line_marker = Marker()
        line_marker.header.frame_id = "odom"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        line_marker.scale.x = 0.01
        line_marker.color.a = 1.0
        line_marker.color.r = 1.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0

        start_point = Point()
        start_point.x = robot_position.x
        start_point.y = robot_position.y
        start_point.z = robot_position.z

        line_marker.points.append(start_point)
        line_marker.points.append(self.target_point)

        self.marker_pub.publish(line_marker)

    def control(self):
        if not self.target_set:
            return

        #self.get_logger().info("Moving...")
        vel = Twist()
        theta_target = math.atan2(self.delta_y,self.delta_x)
        angle_diff = theta_target - self.yaw

        if self.delta_distance > 0.2:
            if (self.state == "MOVE_TARGET"):
                self.get_logger().info("Move to target...")
                self.velocity = 0.1
                self.omega =0.2
                if abs(angle_diff) > 0.1:
                    vel.angular.z = self.omega if angle_diff > 0 else -self.omega
                else:
                    vel.linear.x = self.velocity
            elif (self.state == "TURN_RIGHT"):
                if(self.move_forward == 1):
                    vel.linear.x = 0.1
                elif (self.rotate == 1):
                    vel.angular.z = -self.omega
                else:
                    pass
            elif (self.state == "TURN_LEFT"):
                if(self.move_forward == 1):
                    vel.linear.x = 0.1
                elif (self.rotate == 1):
                    vel.angular.z = self.omega
                else:
                    pass
            elif (self.state == "FOWARD"):
                if(self.move_forward == 1):
                    vel.linear.x = 0.1
                else:
                    vel.linear.x = 0.0
            else:
                self.state = "CHECK"
            
        else:
            self.velocity = 0.0
            self.omega = 0.0
        #vel.linear.x = min(self.velocity,self.delta_distance)
        #vel.angular.z = self.omega
        self.velocity_pub.publish(vel)

    def lidar_callback(self, msg):
        
        #Lidar point = 360
        lidar_ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_inc = msg.angle_increment
        deg20 = int((20/360) * len(lidar_ranges))
        deg340 = int((340/360) * len(lidar_ranges))
        deg85 = int((85/360) * len(lidar_ranges))
        deg95 = int((95/360) * len(lidar_ranges))
        deg275 = int((275/360) * len(lidar_ranges))
        deg265 = int((265/360) * len(lidar_ranges))
        left_distance = min(min(lidar_ranges[0:deg20]),10.0)
        right_distance = min(min(lidar_ranges[deg340:]),10.0)
        front_distance = min(min(lidar_ranges[0:deg20] + lidar_ranges[deg340:]),10.0)
        left90_dis = min(min(lidar_ranges[deg85:deg95]),10.0)
        right270_dis = min(min(lidar_ranges[deg265:deg275]),10.0)
        around_distance = min(min(lidar_ranges),10.0)
        
        safe_distance = 0.3

        #self.get_logger().info(f"Angle Min : {angle_min} | Angle Max : {angle_max}")
        #self.get_logger().info(f"Distance {right_distance}")
        if(self.state == "CHECK"):
            if left_distance < safe_distance:
                self.state = "TURN_RIGHT"
                self.get_logger().info("Obstacle detected! Truning right...")
            elif right_distance < safe_distance:
                self.state = "TURN_LEFT"
                self.get_logger().info("Obstacle detected! Truning left...")
            elif front_distance < safe_distance:
                self.state = "FOWARD"
                self.get_logger().info("Moving foward...")
            else:
                self.move_forward = 1
                self.rotate = 0
        
        if (self.state == "TURN_RIGHT"):
            
            self.backup = "right"
            if left_distance > safe_distance:
                self.state = "FOWARD"
            else:
                self.move_forward = 0
                self.rotate = 1

        if (self.state == "TURN_LEFT"):
            self.backup = "left"
            if right_distance > safe_distance:
                self.state = "FOWARD"
                
            else:
                self.move_forward = 0
                self.rotate = 1

        if (self.state == "FOWARD"):
            self.get_logger().info("Path clear! Moving forward...")
            if (around_distance > 0.35):
                self.state = "MOVE_TARGET"
            elif front_distance > safe_distance:
                self.move_forward = 1
                self.rotate = 0
            else:
                self.state = "CHECK"

        if (self.state == "MOVE_TARGET"):
            
            if (left_distance < safe_distance) or (right_distance < safe_distance):
                self.state = "CHECK" 
            else:
                self.move_forward = 1
                self.rotate = 0
       
################################################################################

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()