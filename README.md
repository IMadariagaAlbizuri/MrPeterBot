# MrPeterBot 🤖
> *Inspired by Mr. Peterson from Hello Neighbour* 🏠

Autonomous car robot that detects troublesome 
neighbours and comes after you. 🚨

## Stack
ROS2 Jazzy · Gazebo Harmonic · Nav2 · 
slam_toolbox · C++ · Python · YOLO v26

## Usage

### Launch simulation
```bash
ros2 launch mr_peterbot_bringup display.launch.py
```

### Manual control (quick test)
```bash
# Moving Forwards
ros2 topic pub /velocity std_msgs/msg/Float64 "data: 0.2" --rate 50
# Steering
ros2 topic pub /steering_angle std_msgs/msg/Float64 "data: 0.3" --rate 50
```

## Roadmap
- [x] ROS2 workspace setup
- [x] Ackermann steering robot URDF from scratch
- [x] Gazebo Harmonic simulation
- [x] ros2_control — Ackermann steering + rear wheel drive
- [x] Ackermann control node (C++)
- [x] LQR Controller (v, θ) added
- [ ] Xbox joystick controller
- [ ] 2D SLAM with slam_toolbox
- [ ] Autonomous navigation with Nav2
- [ ] Person detection with YOLO v26
- [ ] Pursuit behavior 😈
- [ ] Siren sound on detection 🔊

## Acknowledgements
Ackermann steering architecture inspired by [Lucas Mazzetto's Ackermann Steering Vehicle Simulation](https://workabotic.com/2025/ackermann-steering-vehicle-simulation/) — great reference for ROS 2 + Gazebo Harmonic integration of 4 wheels mobile vehicles.

## Warning
Do not deploy in close neighbourhoods.
MrPeterBot shows no mercy. 🏃