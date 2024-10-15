import rclpy
from rclpy.node import Node

from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose2D

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from math import sqrt, atan2, cos, sin
import cv2
import numpy as np
from random import randrange
import time
from skimage.draw import line

class RRT(Node):
    def __init__(self, K=2000, dq=20):
        Node.__init__(self, "rrt_node")

        """ Attributes """
        self.robot_pose = Pose2D()  #Current pose of the robot: self.robot.x, self.robot.y, robot.theta(last one is useless)
        self.path = []  #Path containing the waypoints computed by the RRT in the image reference frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  #used to get the position of the robot
        self.nodes = []
        self.edges = []
        self.path = []
        self.reduced_path= []
        self.K = K
        self.dq = dq
        self.smooth_path = []
        """ Publisher and Subscriber """
        self.create_subscription(PoseStamped, "/goal_pose", self.goalCb, 1)
        self.path_pub = self.create_publisher(Path, "/path", 1)

        """ Load the map and create the related image"""
        self.getMap()
        self.drawMap()
        
        
    
    def Rand_free_conf(self):
        "Return x and y, a random configuration"
        map_data = self.image_grey.T #image in grey scale
        while True:
            x = int(np.random.uniform(0, self.map.info.width-1)) #x
            y = int(np.random.uniform(0, self.map.info.height-1)) #y
            if map_data[x][y] == 255: #if it's a free space
                return [x, y] #returning the random configuration
            
    def nearest_vertex(self, q_rand):
        "Find the node in the tree closest to the random configuration q_rand"
        nearest_node = None
        min_dist = float('inf') #initializing the distance ot infinity
        for node in self.nodes: #for all the nodes
            dist = np.sqrt((q_rand[0]-node[0])**2+(q_rand[1]-node[1])**2) #computing the distance
            if 0 < dist < min_dist: #if dist inferior of the minimum distance then
                min_dist = dist #minimum distance takes the distance of the node
                nearest_node = node #the nearest node is replkaced by the actual node
        return nearest_node #returning the nearest node 
    
    def new_conf(self, q_near, q_rand):
        "return a new configuration avoiding the obstacles"
        q_new = None
        map_data = self.image_grey.T  #image in grey scale
        while(np.all(q_new)==None):#while we don't have a q_new
            direction = np.array(q_rand) - np.array(q_near)
            direction = direction.astype(float)
            direction /= np.linalg.norm(direction) #generating the direction if the new conf from q_near
            q_new_test = np.array(q_near) + self.dq * direction #computing q_new
            self.get_logger().info(f"q_new_test : {q_new_test}")
            if (q_new_test[0]<self.map.info.width-1) and (q_new_test[1]<self.map.info.height-1):
                if map_data[int(q_new_test[0])][int(q_new_test[1])] == 255: #checking if it's in a free space (255 in the image is a white block)
                    q_new = q_new_test
                    return q_near, [int(q_new[0]), int(q_new[1])] #returning if the q_new is in free space
            q_rand = self.Rand_free_conf() #computing an other q_rand because q_new in obstacle
            q_near = self.nearest_vertex(q_rand) #computing an other q_near because q_new in obstacle
            
    
    def __del__(self):
        """ Called when the object is destroyed """
        cv2.destroyAllWindows() #destroy all the OpenCV windows you displayed


    # **********************************   
    def getMap(self):
        """ Method for getting the map """
        #DO NOT TOUCH
        map_cli = self.create_client(GetMap, "map_server/map")
        while not map_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for map service to be available...")
        map_req = GetMap.Request()
        future = map_cli.call_async(map_req)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    self.map = future.result().map  #OccupancyGrid instance!
                    self.get_logger().info("Map loaded !")
                except Exception as e:
                    self.get_logger().info(f"Service call failed {e}")
                return
        
    def reset_image(self):
        "This function resets the self.image"
        map_data = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        map_data = np.flipud(map_data)#plotting the map in the good way
        self.image = np.zeros((self.map.info.height, self.map.info.width), dtype=np.uint8)
        # Iterate over the map data and set the corresponding pixels in the image
        for i in range(self.map.info.height):#height of the map
            for j in range(self.map.info.width):#width of the map
                if map_data[i, j] == 0: #if it's a free space
                    self.image[i, j] = 255  # Set white pixels for free space
                elif map_data[i, j] == 100: #if it's an obstacle
                    self.image[i, j] = 0  # Set black pixels for obstacles
        if len(self.image.shape) == 2 or self.image.shape[2] == 1:
            self.image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)#adding colors in the image


    def drawMap(self):
        """ Method for drawing the map """
        # Convert the occupancy grid to a numpy array
        map_data = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        map_data = np.flipud(map_data)#plotting the map in the good way
        self.image_grey = np.zeros((self.map.info.height, self.map.info.width), dtype=np.uint8)# Create a black image with the same size as the map
        self.image = np.zeros((self.map.info.height, self.map.info.width), dtype=np.uint8)
        # Iterate over the map data and set the corresponding pixels in the image
        for i in range(self.map.info.height):#height of the map
            for j in range(self.map.info.width):#width of the map
                if map_data[i, j] == 0: #if it's a free space
                    self.image[i, j] = 255  # Set white pixels for free space
                    self.image_grey[i, j] = 255
                elif map_data[i, j] == 100: #if it's an obstacle
                    self.image[i, j] = 0  # Set black pixels for obstacles
                    self.image_grey[i, j] = 255

        if len(self.image.shape) == 2 or self.image.shape[2] == 1:
            self.image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)#adding colors in the image
        
        self.expand_obstacles()
        #Display the image
        #cv2.imshow("Map", self.image_grey)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
    # **********************************

    def get_robot_pose(self):
        """ Get the current position of the robot """
        #DO NOT TOUCH
        try:
            trans = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time()
            )
            self.robot_pose.x = trans.transform.translation.x
            self.robot_pose.y = trans.transform.translation.y
        except TransformException as e:
            self.get_logger().info(f"Could not transform base_footprint to map: {e}")

    
    # **********************************
    def goalCb(self, msg):
        self.nodes = []
        self.edges = []
        self.path = []
        self.reduced_path = []
        self.smooth_path = []

        self.get_robot_pose()
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        start_x = self.robot_pose.x
        start_y = self.robot_pose.y
        self.goal_pose = msg.pose
        self.x_goal_image, self.y_goal_image = self.map_to_image_frame(goal_x, goal_y)
        self.get_logger().info(f"goal position in the image frame : {self.x_goal_image, self.y_goal_image}")
        self.x_start_image, self.y_start_image = self.map_to_image_frame(start_x, start_y)
        self.get_logger().info(f"Starting position in the image frame : {self.x_start_image, self.y_start_image}")

        cv2.rectangle(self.image, (self.x_goal_image-3 ,self.y_goal_image+3), (self.x_goal_image+2 ,self.y_goal_image-2), (0, 0, 0), -1) #plotting the goal on the map
        cv2.rectangle(self.image, (self.x_start_image-3 ,self.y_start_image+3), (self.x_start_image+2 ,self.y_start_image-2), (0, 0, 0), -1) #plotting the start on the map

        self.run([self.x_start_image, self.y_start_image])#running the RRT algorithm with the initial configuration
        self.get_logger().info(f"nodes : {self.nodes}")
        if self.nodes != []: #if a path is found
            for edge in self.edges:
                cv2.line(self.image, (edge[1]), (edge[0]), (0, 255, 0), thickness=1, lineType=8)#plotting a line for each edge on the map
            for node in self.nodes:
                cv2.rectangle(self.image, (node[0],node[1]+1), (node[0]+1,node[1]), (0, 0, 255), -1)
            #self.get_logger().info(f"edges : {self.edges}")

            self.get_full_path()
            self.get_logger().info(f"Full path : {self.path}")
            #self.draw_line(self.path, 0, 255, 0, 2) #plotting lines for the path edges
            for node in self.path: #plotting the points of the path
                cv2.rectangle(self.image, (node[0],node[1]+1), (node[0]+1,node[1]), (255, 0, 0), -1)

            #self.reset_image()
            self.draw_line(self.path, 0, 255, 0, 2)  #plotting lines for the path edges
            self.publishPath()
            """cv2.imshow("Path", self.image) #plotting the path
            cv2.waitKey(0) 
            cv2.destroyAllWindows()"""
            self.get_reduced_path()
            self.get_logger().info(f"reduced path : {self.reduced_path}")
            self.reduced_path = self.interpolate_path(self.reduced_path, 12) #interpolating the path
            self.get_logger().info(f"interpolated path : {self.reduced_path}")
            self.draw_line(self.path, 0, 255, 0, 2) 
            self.draw_line(self.reduced_path, 0, 0, 255, 1)
            """cv2.imshow("Reduced path", self.image) #plotting the reduced path
            cv2.waitKey(0) 
            cv2.destroyAllWindows()"""
            self.get_smooth_path(self.reduced_path)
            self.draw_line(self.path, 0, 255, 0, 2)  #plotting lines for the path edges
            self.draw_line(self.smooth_path, 255, 0, 0, 1) #plotting lines for the path edges
            """cv2.imshow("Smoothed path", self.image) #plotting the smoothed path
            cv2.waitKey(0)  
            cv2.destroyAllWindows()"""
            self.reset_image() #reseting the image at the end
            self.publishPath() #sending the path
        

    def draw_line(self,path, B,G,R, size):
        """This function add a draw a line or path on the self.image
        It takes in parameters the path of points (the points that you want to rely), 
        The color (Blue, Green, Red) of the lines,
        the size of the lines"""
        for i in range(len(path) - 1): #recreating edges from the nodes of the path
            start_point = (path[i][0], path[i][1])
            end_point = (path[i+1][0], path[i+1][1])
            cv2.line(self.image, start_point, end_point, (B, G, R), size)  #plotting lines for the path edges


    def get_full_path(self):
        "This function is used to retrieve the full path from all the nodes that we got from the RRT algorithmm"
        edges = self.edges #taking the list of edges

        x_goal = self.x_goal_image #taking the coordinates of the goal
        y_goal = self.y_goal_image

        path_test = []
        
        path_test.append([x_goal, y_goal])
        x = self.x_start_image
        y = self.y_start_image
        #self.get_logger().info(f"edges : {edges}")
        while (path_test[-1]!=[x, y]):
            for edge in edges:
                #self.get_logger().info(f"test : {edge[1], path_test[-1]}")
                if edge[1]==path_test[-1]:
                    path_test.append(edge[0])
                    #self.get_logger().info(f"path : {path_test}")
                    break
        path_test.reverse()
        self.path = path_test

    #The following function works but is not optimal, it takes more time than the function bellow this one
    """def is_edge_valid(self, q_near, q_new):
        #this function take in input a q_near and a q_new. 
        #It return False if the line formed by q_near and q_new passes through an obstacle.
        #Otherwise, it returns True.
        map_data = self.image_grey_dilated.T
        # Extract x and y coordinates
        x_coords = [q_near[0], q_new[0]]
        y_coords = [q_near[1], q_new[1]]
        # Calculate the slope (m) and intercept (b) of the line y = mx + b
        m, b = np.polyfit(x_coords, y_coords, 1)

        x = []
        y = []
        for x_float in np.arange(min(x_coords), max(x_coords), 0.01):
            y_val = np.round(m * x_float + b).astype(int)
            x_val = np.round(x_float).astype(int)
            x.append(x_val)
            y.append(y_val)
        x = np.round(np.arange(min(x_coords), max(x_coords), 0.01)).astype(int)
        points = list(zip(x, y))
        #points = list(set(points))

        obstacle_point = []

        for point in points:
            
            if map_data[point[0]][point[1]] == 255:#if point in free space
                obstacle_point.append(True)
            else:
                obstacle_point.append(False)#if point in obstacle
                return False
        return True"""

    def expand_obstacles(self, dilation_size=20):
        """ Dilate the obstacles so that the trajectory passes farther from the obstacles """
        kernel = np.ones((dilation_size, dilation_size), np.uint8)
        self.image_grey_dilated = cv2.erode(self.image_grey, kernel, iterations=1) #expanding the edges by eroding the free spaces

    def is_edge_valid(self, q_near, q_new):
        "This function returns True in there's no obstacles between two nodes"
        x0, y0 = q_near
        x0 = int(x0)
        y0 = int(y0)
        x1, y1 = q_new
        x1 = int(x1)
        y1 = int(y1)
        rr, cc = line(x0, y0, x1, y1) #all the points between two points
        return np.all(self.image_grey_dilated.T[rr, cc] == 255) #looking in the image if it's a free space for each point

    def map_to_image_frame(self, x, y):
        "This function returns map fram coordinates to map frame coordinates"
        transformation_matrix1 = np.array([
            [1, 0,0,-self.map.info.origin.position.x],
            [0, 1,0,-self.map.info.origin.position.y],
            [0, 0,1,0],
            [0, 0,0,1]
        ])
        
        transformation_matrix2 = np.array([
            [1, 0,0,0],
            [0, -1,0,self.map.info.height*0.05],
            [0, 0,-1,0],
            [0, 0,0,1]
        ])
        compute = transformation_matrix2@transformation_matrix1
        compute2 = compute@(np.array([x,y,0,1]).transpose())
        image_frame = (compute2/(0.05))
        
        return int(image_frame[0]), int(image_frame[1])

    def image_to_map_frame(self, x, y):
        "this function converts image frame coordinates to map frame coordinates"
        transformation_matrix1 = np.array([
            [1, 0,0,-self.map.info.origin.position.x],
            [0, 1,0,-self.map.info.origin.position.y],
            [0, 0,1,0],
            [0, 0,0,1]
        ])
        
        transformation_matrix2 = np.array([
            [1, 0,0,0],
            [0, -1,0,self.map.info.height*0.05],
            [0, 0,-1,0],
            [0, 0,0,1]
        ])
        compute = np.linalg.inv(transformation_matrix1)@np.linalg.inv(transformation_matrix2)
        map_frame = compute@(np.array([x*0.05,y*0.05,0,1]).transpose())

        return (map_frame[0]), (map_frame[1])

    # **********************************
    def run(self, q_init):
        q_new = 0
        q_rand = 0
        q_near = 0
        self.get_logger().info(f"q_init : {q_init}")
        self.nodes.append(q_init) #adding starting point
        for i in range(self.K): #for i in the range of the max iterations
            self.get_logger().info(f"n iter : {i}") #plotting the number of iterations
            q_rand = self.Rand_free_conf() # Generate a random configuration
            q_near = self.nearest_vertex(q_rand)  # Find the nearest node in the tree to q_rand
            q_near, q_new = self.new_conf(q_near, q_rand)  # Generate a new configuration towards q_rand
            if q_new not in self.nodes: #if the new conf not already in the nodes
                if self.is_edge_valid(q_new, q_near): #if no obstacles between now conf and nearest node
                    self.nodes.append(q_new)  # Add the new configuration to the tree
                    self.edges.append([q_near, q_new]) #adding the nearest node and new conf to make the edges
            # Check if the new configuration is close to the goal
            goal = [self.x_goal_image, self.y_goal_image]
            if self.is_edge_valid(q_new, q_near): #if no obstacles between new conf and nearest conf
                if self.is_edge_valid(q_new, goal): #if no obstacle between new conf and goal then
                    if np.linalg.norm(np.array(q_new) - np.array(goal)) < self.dq: #if new conf within the range of the goal
                        last_edge = self.edges[-1][1] 
                        self.edges.append([last_edge, [self.x_goal_image, self.y_goal_image]]) #adding the goal to the path
                        return self.edges, self.nodes  # return the tree if the goal is reachedd
        self.get_logger().info(f"Le goal n'a pas été trouvé") #if max iteration
        self.edges = []
        self.nodes = []
        return self.edges, self.nodes
        
    # **********************************

    def de_casteljau(self, control_points, t):
        """ De Casteljau's Algorithm for evaluating Bézier curves. """
        n = len(control_points)
        if n == 1:
            return control_points[0]
        else:
            new_points = []
            for i in range(n - 1):
                x = (1 - t) * control_points[i][0] + t * control_points[i + 1][0] #x
                y = (1 - t) * control_points[i][1] + t * control_points[i + 1][1] #y
                new_points.append((x, y)) #adding the new point
            return self.de_casteljau(new_points, t)
    
    def interpolate_path(self, path, num_points=10):
        """
        Interpolates the path by adding intermediate points between each pair of points.
        This function is used to less smooth the path and to follow more the reduced path so we can
        avoid the walls.
        """
        interpolated_path = []

        for i in range(len(path) - 1):
            start_point = path[i]#adding the first point
            end_point = path[i + 1]
            for t in np.linspace(0, 1, num=num_points, endpoint=False): # Generate intermediate points between two nodes of the path
                x = (1 - t) * start_point[0] + t * end_point[0] #x
                y = (1 - t) * start_point[1] + t * end_point[1] #y
                interpolated_path.append([int(x), int(y)]) #adding points to the interpolated path

        interpolated_path.append(path[-1])  # Add the last point to the reduced path

        return interpolated_path
    

    def get_smooth_path(self, path):
        """ Smooth the path using De Casteljau's Algorithm. """
        smooth_path = []
         #Create a new list of control points with midpoints
        control_points = []
        for i in range(len(path) - 1):
            control_points.append(path[i])
            midpoint = ((path[i][0] + path[i + 1][0]) / 2, (path[i][1] + path[i + 1][1]) / 2)
            control_points.append(midpoint)
        control_points.append(path[-1])  # add the last point
        for t in np.linspace(0, 1, num=35):  # number of points created
            point = self.de_casteljau(control_points, t)
            smooth_path.append([int(point[0]), int(point[1])])#adding to the smooth path
        self.smooth_path = smooth_path
        return self.smooth_path
    # **********************************
        
    def get_reduced_path(self):
        "This function is used to get a reduced path from a path"
        if not self.path: #if path empty
            return []

        reduced_path = [self.path[0]] #adding first point of the path
        current_index = 0

        while current_index < len(self.path) - 1: #while path has more than the last element
            next_index = current_index + 1 #updating index
            for i in range(len(self.path) - 1, current_index, -1): #for the nodes still in the path
                if self.is_edge_valid(self.path[current_index], self.path[i]): #if we can draw a line which don't passes through a wall
                    next_index = i #update index
                    break

            reduced_path.append(self.path[next_index]) #adding the reduced point
            current_index = next_index #update index
        self.reduced_path = reduced_path 
        return self.reduced_path #returning the full path

    def publishPath(self):
        """ Send the computed path so that RVIZ displays it """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        path_rviz = []
        for pose_img in self.smooth_path: #for the points in the smooth path
            pose = PoseStamped()
            point = self.image_to_map_frame(pose_img[0], pose_img[1]) #converting each point to map frame
            pose.pose.position.x = point[0] #x
            pose.pose.position.y = point[1] #y
            path_rviz.append(pose) #adding the pose to the rviz path
        msg.poses = path_rviz
        self.path_pub.publish(msg) #sending path to display it


def main():
    #DO NOT TOUCH
    rclpy.init()

    node = RRT()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()