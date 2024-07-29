## ROS2 mycobot280m5 Gazebo Adaptation

### 1. Environment Setup

The project uses Ubuntu 22.04 with the corresponding ROS2-Humble version. You can refer to the installation guide on the [Elephant Robotics official website](https://docs.elephantrobotics.com/docs/mycobot-m5-en/12-ApplicationBaseROS/12.2-ROS2/12.2.1-InstallationOfROS2.html) for the installation process. After installation, you can validate the setup by following the instructions for [robot arm control](https://docs.elephantrobotics.com/docs/mycobot-m5-en/12-ApplicationBaseROS/12.2-ROS2/12.2.4-rivzIntroductionAndUse/myCobot-280.html).

Additionally, you need to install Gazebo and MoveIt for which the instructions are provided below.

**Installing Gazebo**

```
sudo apt install gazebo
sudo apt install ros-humble-gazebo-*
```

**Installing moveit-setup-assistant**

```
sudo apt install ros-humble-moveit-setup-assistant
```

**Installing Other MoveIt2 Components**

```
sudo apt install ros-humble-moveit-*
```

To verify if Gazebo is set up correctly, run `gazebo` in the terminal, and you will see the following image

![10png](./image/10.png?msec=1721980139454)

### 2. Usage

Navigate to the ROS2 workspace in the terminal:

```
cd ~/colcon_ws
```

#### 2.1 Slider Control

To control the arm joints using sliders, run:

```
ros2 launch mycobot_280_gazebo_moveit slider_control_gazebo.launch.py
```

You will see the following user interface and you can use the slider to control the pose of the robotic arm

![12png](./image/12.png?msec=1721980413413)

#### 2.2 Model Follow

To load the 280m5 arm model in Gazebo and control its pose by manipulating the physical arm, run:

```
ros2 launch mycobot_280_gazebo_moveit follow_display_gazebo.launch.py
```

#### 2.3 Keyboard Control

Control the arm model in Gazebo using the keyboard as follows:

1. Launch the keyboard teleoperation node:

```
ros2 launch mycobot_280_gazebo_moveit teleop_keyboard_gazebo.launch.py
```

2. In a new terminal, run the teleop keyboard script:

```
cd ~/colcon_ws/src/mycobot_ros2/mycobot_280/mycobot_280_gazebo_moveit/mycobot_280_gazebo_moveit
python3 teleop_keyboard.py
```

Then you will see the otuput as following:

```bash
Mycobot_280m5 Teleop Keyboard Controller

---------------------------

Movimg options (control the angle of each joint):

    w: joint2_to_joint1++   s: joint2_to_joint1--

    e: joint3_to_joint2++   d: joint3_to_joint2--

    r: joint4_to_joint3++   f: joint4_to_joint3--

    t: joint5_to_joint4++   g: joint5_to_joint4--

    y: joint6_to_joint5++   h: joint6_to_joint5--

    u: joint6output_to_joint6++ j: joint6output_to_joint6--



Other:

    1 - Go to init pose

    2 - Go to home pose

    3 - Resave home pose

    q - Quit
```

At this point you can follow the above instructions to operate the robot arm via keyboard input, **NOTE** that this terminal needs to read your keyboard input in real time, so please switch this terminal to the forefront (before the keyboard input, click on the terminal), so that you can control the robotic arm model through the keyboard input