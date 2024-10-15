import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist, PoseStamped
from nav_msgs.msg import Path

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

from math import sqrt, pi, atan2, sin, cos
import numpy as np


class Motion(Node):
    def __init__(self):
        Node.__init__(self, "motion_node")
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_pose = Pose2D()  #Current pose of the robot: self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta
        self.goal_received, self.reached = False, False #new goal received; whether the goal has been reached

        self.create_subscription(Path, "/path", self.plannerCb, 1)  #Subscriber to get the path computed by the RRT node
        self.robot_path_pub = self.create_publisher(Path, "/robot_path", qos_profile=1) #Publisher to send real robot's path to RVIZ
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)  #Publisher to send velocity messages to Gazebo
        self.create_timer(0.1, self.run)    #run the navigation strategy @ 10Hz (DO NOT CHANGE the timer frequency) 
        # PID controller parameters for angular velocity
        self.kp_angular = 1.0
        self.ki_angular = 0.0
        self.kd_angular = 0.0

        self.prev_angular_error = 0.0
        self.sum_angular_error = 0.0


    def get_robot_pose(self):
        """ Get the current position of the robot (continuous) in the map reference frame """
        #DO NOT TOUCH
        try:
            trans = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time()
            )
            self.robot_pose.x = trans.transform.translation.x
            self.robot_pose.y = trans.transform.translation.y
            quat = (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
            self.robot_pose.theta = euler_from_quaternion(quat)[2]
        except TransformException as e:
            self.get_logger().info(f"Could not transform base_footprint to map: {e}")


    def plannerCb(self, msg):
        """ Callback method to get incoming path computed by the RRT node """
        #DO NOT TOUCH
        self.reached, self.goal_received = False, True
        self.path = msg.poses[1:]   #remove the robot's pose
        self.inc = 0    #current index of the waypoint to reach from the path

        #Real path followed by the robot
        self.real_path_msg = Path()
        self.real_path_msg.header.frame_id = "map"
        self.real_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.real_path = []

    def follow_path(self):
        if len(self.path) == 0:
            self.reached = True
            self.get_logger().info("Path completed")
            self.linear = 0.0
            self.angular = 0.0
            self.send_velocities()
            return

        goal = self.path[0]
        if np.linalg.norm(goal.pose.position.x - self.robot_pose.x) < 0.5 and np.linalg.norm(goal.pose.position.y - self.robot_pose.y) < 0.5:
            self.path.pop(0)
            self.get_logger().info(f"Goal reached, {len(self.path)} remaining")
            return
    
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y

        # Calculate the angle to the goal
        delta_x = goal_x - self.robot_pose.x
        delta_y = goal_y - self.robot_pose.y
        angle_to_goal = atan2(delta_y, delta_x)
        err = angle_to_goal - self.robot_pose.theta

        # Limit the error to (-pi, pi):
        phi_err = atan2(sin(err), cos(err))

        # Set the linear and angular velocities
        self.linear = 0.5  # Set a constant linear velocity
        self.angular = phi_err
        # self.get_logger().info(f"Angle to goal:{angle_to_goal * 180 / pi}")
        # self.get_logger().info(f"Angle to robot:{- self.robot_pose.theta * 180 / pi}")
        self.send_velocities() # Send the velocities


    def follow_path_PID(self):
        if len(self.path) == 0: 
            self.reached = True 
            self.get_logger().info("Path completed") 
            self.linear = 0.0 #if path is reached we stop the robot
            self.angular = 0.0 
            self.send_velocities() 
            self.goal_received = False
            return 

        goal = self.path[0] 
        goal_x = goal.pose.position.x 
        goal_y = goal.pose.position.y 

        
        delta_x = goal_x - self.robot_pose.x 
        delta_y = goal_y - self.robot_pose.y 
        distance_to_goal = sqrt(delta_x**2 + delta_y**2) # Calculate the distance to the goal
        angle_to_goal = atan2(delta_y, delta_x) # Calculate the angle to the goal

        angular_error = angle_to_goal - self.robot_pose.theta # Calculate the angular error
        angular_error = atan2(sin(angular_error), cos(angular_error)) # Normalize the angular error to the range (-pi, pi)

        
        self.sum_angular_error += angular_error # PID control for angular velocity
        angular_derivative = angular_error - self.prev_angular_error 
        self.angular = (self.kp_angular * angular_error + 
                        self.ki_angular * self.sum_angular_error + 
                        self.kd_angular * angular_derivative) *4 #we can adjust the speed of the rotation by the scalar at the end
        self.prev_angular_error = angular_error #updating the error

        self.linear = 1.2 if distance_to_goal > 0.5 else 0.0 # Set a constant linear velocity to move towards the goal

        self.get_logger().info(f"Distance to goal: {distance_to_goal:.2f}") 
        self.get_logger().info(f"Angle to goal: {angle_to_goal * 180 / pi:.2f}") 

        self.send_velocities() # Send the velocities
        
        if distance_to_goal < 0.5: # Check if we are close enough to the goal to consider it reached
            self.path.pop(0) 
            self.get_logger().info(f"Waypoint reached, {len(self.path)} remaining") 


    def run(self):
        """ Main method called periodically to reach the waypoints """
        if not self.reached and self.goal_received:
            self.get_robot_pose()

            #self.follow_path() #Without PID
            self.follow_path_PID()

            self.publish_path()  # this is the last instruction in this if statement!!

    
    def send_velocities(self):
        """ Send computed linear and angular velocities """
        #DO NOT TOUCH
        self.linear = self.constrain(self.linear, min=-2.0, max=2.0)
        self.angular = self.constrain(self.angular, min=-3.0, max=3.0)

        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear
        cmd_vel.angular.z = self.angular
        self.vel_pub.publish(cmd_vel)


    def constrain(self, val, min=-2.0, max=2.0):
        """ Make sure a value is within the range [min, max] """
        #DO NOT TOUCH
        if val < min:
            return min
        elif val > max:
            return max
        return val


    def publish_path(self):
        """ Publish the real path followed by the robot """
        #DO NOT TOUCH
        pose = PoseStamped()
        pose.pose.position.x = self.robot_pose.x
        pose.pose.position.y = self.robot_pose.y
        self.real_path.append(pose)
        self.real_path_msg.poses = self.real_path
        self.robot_path_pub.publish(self.real_path_msg)



def main():
    rclpy.init()

    node = Motion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()