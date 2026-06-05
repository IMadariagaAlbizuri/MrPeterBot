# MrPeterBot 🤖

> *Inspired by Mr. Peterson from Hello Neighbour* 🏠

Autonomous car robot that detects troublesome 
neighbours and comes after you. 🚨

## Stack
ROS2 Jazzy · Gazebo Harmonic · Nav2 · 
slam_toolbox · Python · YOLO v26

## Usage

### Launch simulation
```bash
ros2 launch mr_peterbot_bringup display.launch.py
```

### Teleoperation
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard  --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
```

## Roadmap
- [x] ROS2 workspace sketup
- [x] Differential drive robot URDF from scratch
- [x] Gazebo Harmonic simulation
- [x] ros2_control — robot moves 
- [ ] 2D SLAM with slam_toolbox
- [ ] Autonomous navigation with Nav2
- [ ] Person detection with YOLO v26
- [ ] Pursuit behavior 😈
- [ ] Siren sound on detection 🔊

## Warning
Do not deploy in close neighbourhoods.
MrPeterBot shows no mercy. 🏃