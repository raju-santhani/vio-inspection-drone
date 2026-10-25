# VIO-Based Inspection Drone

An inspection-drone platform integrating RealSense-based visual-inertial odometry, NVIDIA Jetson Orin Nano, and an ArduPilot flight controller.

The project focuses on onboard positioning, coordinate-frame conversion, MAVLink integration, and flight-performance evaluation. Ongoing work explores adaptive control for pipeline non-destructive testing (NDT).

## Project Overview

I deployed NVIDIA Isaac ROS Visual SLAM on a Jetson Orin Nano using a RealSense camera to obtain real-time position and orientation estimates.

I then integrated the positioning output with an ArduPilot flight controller through MAVLink, implementing the coordinate-frame conversion required between the ROS output and the flight-controller interface.

Flight performance was evaluated using Vicon motion-capture cameras as an external reference.

## My Contributions

- Deployed the Isaac ROS positioning pipeline on Jetson Orin Nano.
- Integrated a RealSense camera for visual-inertial positioning.
- Implemented coordinate-frame conversion for the ROS-to-ArduPilot interface.
- Communicated external positioning information to the flight controller using MAVLink.
- Evaluated hover accuracy during a continuous 10-minute flight using Vicon motion capture.
- Integrated an external time-of-flight (ToF) sensor, reducing altitude-tracking error by 20% under the tested conditions.

## System Architecture

```text
RealSense Camera
       |
       v
Isaac ROS Visual SLAM
on Jetson Orin Nano
       |
       v
Coordinate-Frame Conversion
       |
       v
MAVLink Communication
       |
       v
ArduPilot Flight Controller
```

Additional components:

- External ToF sensor for altitude measurement.
- Vicon motion-capture system for flight-performance evaluation.

## Hardware

| Component | Hardware Used |
|---|---|
| Companion computer | NVIDIA Jetson Orin Nano |
| Vision sensor | Intel RealSense — model to be added |
| Flight controller | Orange Cube |
| Autopilot firmware | ArduPilot |
| Altitude sensor | External ToF sensor |
| Validation system | Vicon motion-capture cameras |

## Software and Integration

### Visual-Inertial Positioning

The positioning pipeline was deployed following NVIDIA's Isaac ROS Visual SLAM documentation:

https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam

This project uses NVIDIA's existing positioning software. My contributions focus on deployment, drone integration, coordinate conversion, and experimental validation.

### Coordinate-Frame Conversion

The integration converts positioning information from the ROS pipeline into the coordinate conventions required by the selected ArduPilot MAVLink interface.

Implementation details to document:

- Input ROS topic and message type
- Input reference frame and camera/body frames
- Output reference frame and body convention
- Position and orientation transformations
- Camera-to-aircraft mounting transformation
- Timestamp handling

### MAVLink Communication

The converted positioning information is transmitted to the flight controller using MAVLink.

Implementation details to document:

- MAVLink message type
- Communication library or bridge
- Serial or network connection
- Transmission rate
- Relevant ArduPilot parameters

## Experimental Validation

### Hover Evaluation

Hover performance was evaluated during a continuous 10-minute flight using Vicon motion-capture measurements as an external reference.

Supporting documentation will describe:

- Test conditions and commanded hover position
- Time synchronization and coordinate alignment
- Position-estimation error relative to Vicon
- Hover-position error relative to the commanded position
- Error metrics and plots

### Altitude Tracking

An external ToF sensor reduced altitude-tracking error by 20% relative to the baseline configuration under the tested conditions.

Supporting documentation will identify:

- Baseline altitude-estimation configuration
- ToF sensor integration method
- Error metric used
- Baseline and updated error values
- Test conditions and supporting data

## Ongoing Work

Adaptive control is under development to support drone-based non-destructive testing of pipelines.

Pipeline NDT is the intended application; this repository will distinguish completed experiments from ongoing development.

## Repository Contents

- `src/` — custom integration and coordinate-conversion code
- `config/` — configuration files and relevant parameters
- `docs/` — setup, integration, and testing documentation
- `results/` — processed measurements and plots
- `references/` — photographs, diagrams, and video links

## References and Acknowledgements

- NVIDIA Isaac ROS Visual SLAM:
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
- ArduPilot:
  https://github.com/ArduPilot/ardupilot

Upstream software remains the work of its respective contributors.
