## Overview

This project demonstrates how to move a turtlesim robot in a pentagon shape using ROS (Robot Operating System). Developed for participating in **CODING MANIA** in **ROBOWEEK** organised by **Robotics Society, NIT HAMIRPUR**, the project employs client-server messaging to control the robot's movements. The turtlesim is a simple and widely used ROS package that simulates a turtle that can be moved around the screen.

**VIDEO LINK** : https://github.com/Sanjukumar8238/ROS_Turtlesim/blob/main/turtlesim_1920x1080.mp4


## Table of Contents

1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Installation](#installation)
4. [Running the Project](#running-the-project)
5. [Understanding the Code](#understanding-the-code)
6. [Project Structure](#project-structure)
7. [Contributing](#contributing)
8. [License](#license)

## Introduction

In this project, we use ROS to move the turtlesim robot in a pentagon pattern. The project illustrates how to use ROS services and topics for robot control through a client-server architecture. The turtlesim receives commands to move forward and rotate at specific angles to form the vertices of a pentagon.

## Prerequisites

Before you begin, ensure you have the following installed:

- **ROS (Noetic recommended)**
  - Installation guide: http://wiki.ros.org/noetic/Installation/Ubuntu
- **Python 3**
  - Python is used to write the scripts that control the turtle.
- **Git**
  - To clone the project repository.

## Installation

1. **Clone the repository:**

   ```bash
   git clone https://github.com/Sanjukumar8238/ROS_Turtlesim.git
   cd ROS_Turtlesim
   ```

2. **Set up the ROS environment:**

   Source the ROS setup file for your shell:

   ```bash
   source /opt/ros/noetic/setup.bash
   ```

3. **Build the package:**

   Navigate to your ROS workspace and build the package:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

4. **Source your workspace:**

   After building, source the workspace:

   ```bash
   source devel/setup.bash
   ```

## Running the Project

1. **Launch ROS master:**

   First, you need to start the ROS master node:

   ```bash
   roscore
   ```

2. **Run the launch file:**

   In a new terminal, navigate to the `ROS_Turtlesim` directory and launch the project:

   ```bash
   roslaunch ros_turtlesim move_pentagon.launch
   ```

   This will start the turtlesim node, the server node, and the client node. The turtle should now move in a pentagon pattern on the screen.

## Understanding the Code

### `move_pentagon_client.py`

This Python script acts as the client that requests the server to move the turtle in a pentagon pattern. The main function:

- **move_square_client():** Waits for the `move_pentagon` service to become available and then calls it, passing the side length and rotation as parameters.

### `move_pentagon_server.py`

This Python script is the server that handles requests from the client and commands the turtle to move. Key functions include:

- **handle_move_square(req):** Handles the client's request to move the turtle in a pentagon shape. It uses the `move_in_line` and `rotate` functions to achieve this.
- **move_in_line(side_length, vel_msg, pub):** Moves the turtle forward by a specified distance.
- **rotate(vel_msg, pub):** Rotates the turtle by the angle necessary to create a pentagon (72 degrees).

### `move_pentagon.launch`

This launch file automates the process of starting the turtlesim node, the server, and the client. It includes:

- **Turtlesim Node:** The simulation of the turtle.
- **Server Node:** The `move_pentagon_server.py` script.
- **Client Node:** The `move_pentagon_client.py` script.

## Project Structure

```
ROS_Turtlesim/
├── src/
│   ├── move_pentagon_client.py   # Client script to request pentagon movement
│   ├── move_pentagon_server.py   # Server script to handle requests and move the turtle
├── launch/
│   └── move_pentagon.launch      # Launch file to run the entire project
└── README.md                     # Project documentation
```

## Contributing

Contributions are welcome! Please fork this repository and submit a pull request with your changes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

---

This `README.md` file provides a comprehensive guide for beginners on how to set up, run, and understand the project. Let me know if you need any further adjustments!
