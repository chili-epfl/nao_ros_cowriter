nao_ros_cowriter/trajectory_following
=====================================

This package contains a set of scripts useful to interface with naoqi or MoveIt
to generate joint-space trajectories to follow an SVG shape.

It requires MoveIt or pyrobots for the robot inverse kinematics, and softMotion
or cowriter-trajectory-generator for the SVG2traj conversion. 

Tested with ROS Hydro.

Available tools
---------------

- `scripts/place_paper.py`: displays an interactive marker in RViz that is
  shaped as an A4 paper sheet. Used to visually place the SVG trajectory in
  space.

- `scripts/publish_traj`: takes an SVG file as input and publishes it as a ROS
  topic (`/write_traj`). Can also optionally display it in RViz.
  
- `scripts/show_traj_onebyone`: takes an SVG file as input and publishes it 
  as a ROS display topic (`/visualization_markers`) for displaying as in RViz
  as an animation.

- `scripts/nao_write_naoqi.py`: reads a cartesian trajectory from ROS topic 
  `write_traj` and generates the corresponding joint-space trajectory using
  naoqi via pyrobots.

- `scripts/nao_write_moveit.py`: reads a cartesian trajectory from ROS topic 
  `write_traj` and generates the corresponding joint-space trajectory using
  MoveIt (work in progress).

