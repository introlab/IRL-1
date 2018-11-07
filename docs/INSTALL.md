# Workstation Environment Setup to Use IRL-1
This procedure should be used only for a workstation, not the robot's on-board computer.

## 0. Install Ubuntu 16.04 LTS
Download ISO File: https://www.ubuntu.com/download/desktop

## 1. Install ROS Kinetic & Gazebo
From: http://wiki.ros.org/kinetic/Installation/Ubuntu

### 1.1. Add ROS repository to apt
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt update
```

### 1.2. Install ROS
Includes: ROS, Gazebo, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception

Download: 495 MB
Install: 2497 MB

```bash
sudo apt-get install ros-kinetic-desktop-full
```

### 1.3. Initialize rosdep
```bash
sudo rosdep init
rosdep update
```

### 1.4. Environment setup
```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.5. Install dependencies for building packages
Download: 9479 kB
Install: 54.0 MB

```bash
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## 2. Install ROS & Gazebo Dependencies
From: http://gazebosim.org/tutorials?tut=ros_installing

### 2.1. ROS & Gazebo Control
Download: 859 kB + 686 kB
Install: 3,807 kB + 3 MB

```bash
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
sudo apt-get install ros-kinetic-gazebo-ros-control
```

## 3. Validate Installation
```base
roscore
gazebo
```

## 4. Install IRL-1

### 4.1. Create New ROS Workspace
```bash
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/src/
catkin_init_workspace
```

### 4.2. Get repository
```bash
cd ~/catkin_ws/src/
git clone git@github.com:introlab/IRL-1.git
```

### 4.3. Compile
```bash
cd ~/catkin_ws/
catkin_make
```
You should compile everytime code changes

### 4.4. Add Workspace to Environment Variable
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 5. Common Problems

### 5.1. Virtual Machine

#### 5.1.1. VirtualBox
Manually install VirtualBoxGuest tools
```bash
sudo apt-get install virtualbox-guest-dkms
```

#### 5.1.2. VMware
If Gazebo start with the error `VMware: vmw_ioctl_command error Invalid argument.`
```bash
echo "export SVGA_VGPU10=0" >> ~/.bashrc
```
