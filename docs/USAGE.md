# Use IRL-1

## 1. Run Simulation

### 1.1 Launch IRL-1 Model
```bash
roslaunch jn0_gazebo jn0_empty_world.launch
```

### 1.2. Add robot model
In Rviz, on the left sidebar, select Add -> `RobotModel` -> OK

### 1.3 Choose Fixed Frame
In Rviz, on the left sidebar, select Displays -> Global Options -> Fixed Frame -> base_link

# ROS Tips

Test controllers
```bash
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

Send message on topics and more
```bash
rosrun rqt_gui rqt_gui
```
