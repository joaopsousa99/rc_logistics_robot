Made for Cognitive Robotics 2021/2022 by João Sousa and Nuno Marques, Masters Degree in Robotics, A.I. and Control, DEEC @ UC

## Table of contents<!-- omit in toc --> 
- [Required packages](#required-packages)
- [How to connect the computer to the robot and run ROS<a name = "connect_pc_to_robot"></a>](#how-to-connect-the-computer-to-the-robot-and-run-ros)
  - [Set up the connection<a name = "wired_connection"></a>](#set-up-the-connection)
    - [NOTES](#notes)
  - [How to check IP address of the robot<a name = "check_ip"></a>](#how-to-check-ip-address-of-the-robot)
- [How to create and save a map<a name = "save_map"></a>](#how-to-create-and-save-a-map)
  - [In simulation<a name = "save_map_sim"></a>](#in-simulation)
  - [In the real world](#in-the-real-world)
- [How to run the navigate using QR codes](#how-to-run-the-navigate-using-qr-codes)

## Required packages
- [amcl](wiki.ros.org/amcl)
- [gazebo_ros](wiki.ros.org/gazebo_ros)
- [gmapping](wiki.ros.org/gmapping)
- [joy](wiki.ros.org/joy)
- [map_server](wiki.ros.org/map_server)
- [move_base](wiki.ros.org/move_base)
- [robot_state_publisher](wiki.ros.org/robot_state_publisher)
- [rviz](wiki.ros.org/rviz)
- [teleop_twist_joy](wiki.ros.org/teleop_twist_joy)
- [teleop_twist_keyboard](wiki.ros.org/teleop_twist_keyboard)
- [turtlebot3_description](wiki.ros.org/turtlebot3_description)
- [video_stream_opencv](wiki.ros.org/video_stream_opencv)
- [xacro](wiki.ros.org/xacro)
- [zbar_ros](wiki.ros.org/zbar_ros)

## How to connect the computer to the robot and run ROS<a name = "connect_pc_to_robot"></a>

### Set up the connection<a name = "wired_connection"></a>
1. Connect the computer to the robot via ethernet cable or wi-fi. The wi-fi password is scout1234.
2. Configure the robot's IP address in wired network settings.
3. Run `./connect.sh`. The password is `1234`.
4. Run `tmux`.
5. Run `roscore`.
6. Press Ctrl+B and then Ctrl+Shift+5
7. Run `roslaunch nomadic_driver robot.launch`.
8. Open a new terminal in the computer.
9. Run `roscd rc_logistics_robot`.
10. Run `source ./export.sh`. 
11. You can now run any ROS command.

#### NOTES
- Make sure to change the hosts file (/etc/hosts) to contain the correct IP address of orangepipcplus, by adding the line `XXX.XXX.XXX.XXX orangepipcplus`.
- To clear the terminal without having to run `source ./export.sh` again, use `clear -x`.
- In each new window of the computer's terminal, start by running `source ./export.sh` (and make sure you're in the right directory).
- If the `connect.sh` or `export.sh` don't work, edit them to make sure the IP address of the robot is correct.
- Use `sftp://group01@XXX.XXX.XXX.XXX` in the file explorer to access the robot's files. The password is `1234`.

### How to check IP address of the robot<a name = "check_ip"></a>
1. Connect to the robots network, nomadb, or connect to it via ethernet cable.
2. Run `sudo apt install net-tools`.
3. Run `ifconfig` and check the first 3 numbers of the your IP address under `wlp3s0`.
4. Run `sudo snap install nmap`.
5. Run `nmap -sP [first 3 IP address numbers].0/24` and check the IP address of `_gateway`. That is the robots IP address.

## How to create and save a map<a name = "save_map"></a>
### In simulation<a name = "save_map_sim"></a>
1. Run `roscore` in the computer.
2. Open new terminal and run `roslaunch rc_logistics_robot mapping_sim.launch [gamepad:=true]` (leave out the gamepad part to use the keyboard instead).
3. Move the robot around. The deadman switch is X for a DualShock 4.
4. Open a new terminal.
5. Change to the `rc_logistics_robot/maps` directory and run `./save_map.sh map_name`. The map will be saved in the current directory.

### In the real world
1. Run `roscore` in the robot.
2. Open a new terminal.
3. Run `roslaunch nomadic_driver robot.launch`.
4. Run `source ./export.sh` in the computer.
5. Run `roslaunch rc_logistics_robot mapping_real.launch [gamepad:=true]` (leave out the gamepad part to use the keyboard instead).
6. Move the robot around. The deadman switch is X for a DualShock 4.
7. Open a new terminal.
8. Change to the `rc_logistics_robot/maps` directory and run `./save_map.sh map_name`. The map will be saved in the current directory.

## How to run the navigate using QR codes
1. Connect to the robot via SSH.
1. Run `roscore` in the robot.
2. Open a new terminal.
3. Run `roslaunch nomadic_driver robot.launch`.
4. Run `source ./export.sh` in the computer.
5. Run `roslaunch rc_logistics_robot qr_goals_real.launch`.
6. Show a QR code to the webcam of the computer which encodes one of the following strings: GoToA; GoToB; GoToC; GoToD. 