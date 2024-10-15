# Algorithmic Approaches To Path Planning And Path Following In Robotics

## Project Overview

This project focuses on environment mapping, path planning, and motion control using ROS 2. The key objectives are to map an environment, plan a path through it using various algorithms, and control the movement of a robot within that environment. The project includes algorithm implementations such as the Rapidly-Exploring Random Tree (RTT) and De Casteljau’s algorithm for path smoothing, with visualization in RVIZ.

---

##### Table of Contents
- [I. Mapping of an Environment](#i-mapping-of-an-environment)
    - [A. Objectives](#a-objectives)
    - [B. Map an Environment](#b-map-an-environment)
    - [C. Path Planning](#c-path-planning)
    - [D. Coordinate System](#d-coordinate-system)
    
- [II. Path Planning Algorithm Using ROS 2](#ii-path-planning-algorithm-using-ros-2)
    - [A. Objectives](#a-objectives-1)
    - [B. Generated Image](#b-generated-image)
    - [C. RTT Algorithm](#c-rtt-algorithm)
    - [D. Path Reduction](#d-path-reduction)
    - [E. Smoothing Path](#e-smoothing-path)
        - [De Casteljau’s Algorithm](#de-casteljau-algorithm)
    - [F. Displaying Your Path on RVIZ](#f-display-your-path-on-rviz)
    
- [III. Motion Control Using ROS 2](#iii-motion-control-using-ros-2)
    - [A. Objectives](#a-objectives-2)
    - [B. Motion Control](#b-motion-control)
    - [C. Implementing the Controller](#c-implement-the-controller)

---

<a name="i-mapping-of-an-environment"/>

## I. Mapping of an Environment

<a name="a-objectives"/>

### A. Objectives
The goal of this section is to map an environment to enable autonomous navigation. We use ROS 2 tools to capture and represent the environment's layout for further path planning.

<a name="b-map-an-environment"/>

### B. Map an Environment
Here, we demonstrate how to create a map of the environment using ROS 2.

<a name="c-path-planning"/>

### C. Path Planning
We explore various path planning strategies such as RTT to enable a robot to navigate the mapped environment efficiently.

<a name="d-coordinate-system"/>

### D. Coordinate System
Explanation of the coordinate system used in mapping and navigation, ensuring proper alignment with ROS 2 standards.

---

<a name="ii-path-planning-algorithm-using-ros-2"/>

## II. Path Planning Algorithm Using ROS 2

<a name="a-objectives-1"/>

### A. Objectives
This section focuses on implementing a path planning algorithm using ROS 2 to find the most efficient route through the mapped environment.

<a name="b-generated-image"/>

### B. Generated Image
A sample image of the environment map generated in the previous section, used as the basis for path planning.
![image](https://github.com/user-attachments/assets/4a513b9d-7a39-4fd4-b88d-a0dd5a367e23)


<a name="c-rtt-algorithm"/>

### C. RTT Algorithm
We implement the RTT algorithm to generate an initial path. The code and explanation of how the algorithm works are provided.

![image](https://github.com/user-attachments/assets/bbda1b0b-6641-4fea-b74e-453076f179ed)


<a name="d-path-reduction"/>

### D. Path Reduction
After generating the initial path, we apply techniques to reduce its complexity and optimize the path for real-world usage.

<a name="e-smoothing-path"/>

### E. Smoothing Path

<a name="de-casteljau-algorithm"/>

#### - De Casteljau’s Algorithm
To smooth the path, we implement De Casteljau’s algorithm. This section includes the code and results of the smoothing process.

#### - Results
After path smoothing, we show a comparison of the initial and smoothed paths and discuss the improvements.
![image](https://github.com/user-attachments/assets/5e25ff26-cfe2-4ee0-9359-94f582fe1f7e)

<a name="f-display-your-path-on-rviz"/>

### F. Displaying Your Path on RVIZ
The planned and optimized path is visualized using RVIZ, a visualization tool in ROS 2, to verify the path before execution.

---

<a name="iii-motion-control-using-ros-2"/>

## III. Motion Control Using ROS 2

<a name="a-objectives-2"/>

### A. Objectives
The objective here is to control the robot's movement along the planned path using ROS 2.

<a name="b-motion-control"/>

### B. Motion Control
We explore motion control techniques such as PID control to ensure accurate and smooth movement along the generated path.

<a name="c-implement-the-controller"/>

### C. Implementing the Controller
This section provides a detailed explanation of how to implement and test the motion controller in ROS 2.


