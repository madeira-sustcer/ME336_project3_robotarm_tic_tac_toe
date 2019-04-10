# BionicDL-CobotLearning-Project3
The project is illustrated with Aubo-i5 and realsense 400 series. If you are doing the project with a different robot arm, just replacing the aubo_robot directory with you robot arm's ROS package and lanuch the corresponding moveit group node.

# For Franka and Cobotta, you will use the gripper to pick up the pieces and be sure to add table and walls in the urdf to avoid collision for motion planning. please refer to the BionicDL-CobotLearning-Project3/aubo_robot/aubo_description/urdf/aubo_i5_robot.urdf.xacro

# Project Setup
Before running the project, make sure you've installed ROS kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu) and catkin_tools (https://catkin-tools.readthedocs.io/en/latest/installing.html).

Please go to the each package under this repository for detailed explanations and installation guidance. Please install the [Realsense SDK](https://realsense.intel.com/sdk-2/#install) before you run the following steps.

For this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly
If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin build
```

Now that you have a workspace, clone or download this repo into the src directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/ancorasir/BionicDL-CobotLearning-Project3.git
```

Now install missing dependencies using rosdep install:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

Build the project:
```sh
$ cd ~/catkin_ws
$ catkin build aubo_msgs
$ catkin build
```

Add following to your .bashrc file:
```
source ~/catkin_ws/devel/setup.bash
```

# Run the minimax algorithm for tic tac toe game and vision detection of chessboard and pieces
Use the hand eye calibration results you've got from project2 and complete the following steps.
1. Launch the robot's movegroup node to monitor, control and visualize the robot. Please check the ip of you robot and replace the default value below.
```sh
$ roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.1.102
```

2. Publish the handeye transformation to /tf, or you can write the transformation in urdf(aubo_i5_robot.urdf.xacro):
```sh
$ python calculate_publish_handeye_matrix.py
```

3. Complete the python script of minimax for tic tac toe game in /BionicDL-CobotLearning-Project3/aubo_robot/aubo_i5_moveit_config/scripts/game.py

4. Complete the main python script of the robot player in  /BionicDL-CobotLearning-Project3/aubo_robot/aubo_i5_moveit_config/scripts/project_grasp_3x3.py
The default game board is a 3x3 chessboard or you can print you own chessboard. Tasks to be completed include: detect the empty chessboard before starting the game, detect the human's move and update the status of the game, compute robot's optimal move using minimax searching algorithm, detect the pieces to be picked by the robot and execute the move using real robot.
