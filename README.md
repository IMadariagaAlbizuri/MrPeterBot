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

### Manual control (quick test)
```bash
# Rear wheels
ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [5.0, 5.0]"

# Front steering
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.3, 0.3]"
```

## Roadmap
- [x] ROS2 workspace setup
- [x] Ackermann steering robot URDF from scratch
- [x] Gazebo Harmonic simulation
- [x] ros2_control — robot moves
- [ ] Ackermann control node (C++ + Xbox joystick)
- [ ] 2D SLAM with slam_toolbox
- [ ] Autonomous navigation with Nav2
- [ ] Person detection with YOLO v26
- [ ] Pursuit behavior 😈
- [ ] Siren sound on detection 🔊

## Warning
Do not deploy in close neighbourhoods.
MrPeterBot shows no mercy. 🏃