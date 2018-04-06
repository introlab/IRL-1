IRL-1
=====

The main repository for the IRL-1 robot at Université de Sherbrooke's IntRoLab.

Getting Started
---------------

The repository is currently quite empty, but a model of Johnny-0 robot, the
torso part of IRL-1, is available. 

The main dependencies are:

 - ROS Kinetic, with those packages:
   - ros_control_gazebo
   - ros_controllers

Normally, a base install of Ubuntu 16.04 and ROS Kinetic ('Desktop full') plus
ros-kinetic-gazebo-ros-control should be enough to start the model.

To run the Johnny-0 torso in an empty world, run:

```
  roslaunch jn0_gazebo jn0_empty_world.launch
```

For further information on how to use the robot in real life, look into the
'docs' [subfolder](docs/README.md).

History
-------

This public repository is being built from updated packages originally found in
the original (private) IntRoLab SVN repository.
Efforts have been made to conserve as much as commit history as possible.

For any info, please contact François Ferland (francois.ferland@usherbrooke.ca).
