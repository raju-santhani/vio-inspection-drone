# VIO-Based Inspection Drone

An inspection-drone platform integrating RealSense-based visual-inertial odometry (VIO), NVIDIA Jetson Orin Nano, and an ArduPilot flight controller.

The project focuses on onboard positioning, coordinate-frame conversion, MAVLink integration, and flight-performance evaluation. Ongoing work explores adaptive control for pipeline non-destructive testing (NDT).

## Project Overview

I deployed NVIDIA Isaac ROS Visual SLAM on a Jetson Orin Nano using an Intel RealSense camera to obtain real-time position and orientation estimates.

I integrated the positioning output with an ArduPilot flight controller through MAVLink, implementing the coordinate transformations required by the flight-controller interface.

Flight performance was evaluated using Vicon motion-capture cameras as an external reference.

The integration also includes an external time-of-flight (ToF) sensor for altitude measurement and evaluation of altitude-tracking performance.

## My Contributions

- Deployed Isaac ROS Visual SLAM on the Jetson Orin Nano.
- Integrated an Intel RealSense camera for visual-inertial positioning.
- Implemented coordinate-frame conversion for the ROS-to-ArduPilot interface.
- Developed a Python-based MAVLink bridge to transmit external odometry to the Orange Cube flight controller.
- Integrated the VIO output with ArduPilot's external-navigation estimation pipeline.
- Evaluated hover accuracy during a continuous 10-minute flight using Vicon motion capture.
- Integrated an external ToF sensor, reducing altitude-tracking error by 20% under the tested conditions.
- Established a software integration workflow using ROS 2, Docker, Isaac ROS, and `pymavlink`.

## System Architecture

```text
Intel RealSense Camera
        |
        | Stereo Images + IMU
        v
NVIDIA Jetson Orin Nano
        |
        v
Isaac ROS Visual SLAM
        |
        | ROS 2 Odometry
        | /visual_slam/tracking/odometry
        v
VIO-to-MAVLink Bridge
(vio_mavlink_bridge_modified.py)
        |
        | Coordinate-Frame Conversion
        | MAVLink ODOMETRY
        v
Orange Cube Flight Controller
        |
        v
ArduPilot EKF / External Navigation
        |
        v
Position Estimation and Flight Control
```

**Additional components**

- External ToF sensor for altitude measurement.
- Vicon motion-capture system for flight-performance evaluation.
- Ground-control station for monitoring flight-controller status and estimator health.

## Hardware

| Component | Hardware Used |
|---|---|
| Companion computer | NVIDIA Jetson Orin Nano |
| Vision sensor | Intel RealSense D435i |
| Flight controller | CubePilot CubeOrange+ |
| Autopilot firmware | ArduPilot |
| Altitude sensor | External ToF sensor |
| Validation system | Vicon motion-capture cameras |

## Software and Integration

### 1. Visual-Inertial Positioning

The positioning pipeline was deployed following the official NVIDIA Isaac ROS Visual SLAM documentation:

https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam

The implementation uses NVIDIA's existing Visual SLAM software. My contributions focus on deployment, integration with the aircraft's flight controller, coordinate conversion, and experimental validation.

**Software environment**

- Operating system: Ubuntu-based Jetson environment.
- ROS 2: Jazzy.
- Execution environment: Docker with NVIDIA GPU support.
- Visual odometry: Isaac ROS Visual SLAM.
- MAVLink communication: Python with `pymavlink`.

### 2. Launching Isaac ROS Visual SLAM

The following instructions assume that the Jetson Orin Nano has been configured according to the Isaac ROS documentation and that the Docker image `isaac-ros-realsense-vslam:pymavlink` is available.

Open Terminal 1 and launch the container:

```bash
docker run --rm -it \
  --user admin \
  --entrypoint /bin/bash \
  --group-add 20 \
  --runtime nvidia \
  --network host \
  --ipc host \
  --privileged \
  -v /dev:/dev \
  -v /home/tihan/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
  isaac-ros-realsense-vslam:pymavlink
```

Source ROS 2:

```bash
source /opt/ros/jazzy/setup.sh
```

Launch the RealSense Visual SLAM pipeline:

```bash
ros2 launch isaac_ros_visual_slam \
  isaac_ros_visual_slam_realsense.launch.py
```

Wait for the camera streams and Visual SLAM node to initialize.

### 3. Verifying the VIO Output

The bridge subscribes to the following ROS 2 topic:

`/visual_slam/tracking/odometry`

Verify that the topic is available:

```bash
ros2 topic list
ros2 topic info /visual_slam/tracking/odometry
```

Inspect the published odometry:

```bash
ros2 topic echo /visual_slam/tracking/odometry
```

Check the publication rate:

```bash
ros2 topic hz /visual_slam/tracking/odometry
```

The expected message type is `nav_msgs/msg/Odometry`, containing position, orientation, linear velocity, angular velocity, and covariance information.

Confirm that the output is continuous and that position and orientation change consistently with physical movement before proceeding to flight-controller integration.

### 4. Coordinate-Frame Conversion

The VIO-to-MAVLink bridge converts the incoming ROS odometry into the frame conventions configured in the MAVLink message.

The supplied bridge assumes the incoming axes correspond to:

- X: Forward.
- Y: Left.
- Z: Up.

For the implemented FLU-to-FRD sign conversion, the position and linear velocity transformations are:

\[
\begin{aligned}
x_{\mathrm{out}} &= x_{\mathrm{in}}\\
y_{\mathrm{out}} &= -y_{\mathrm{in}}\\
z_{\mathrm{out}} &= -z_{\mathrm{in}}
\end{aligned}
\]

The bridge also transforms angular velocity, quaternion components, and covariance matrices.

It assigns the following MAVLink frame identifiers:

```python
frame_id = mavutil.mavlink.MAV_FRAME_LOCAL_FRD
child_frame_id = mavutil.mavlink.MAV_FRAME_BODY_FRD
```

These transformations must be validated against the actual Isaac ROS world-frame convention, camera mounting orientation, and ArduPilot interface requirements. The camera-to-aircraft mounting transformation must also be considered if the VIO pose is referenced to the camera rather than the flight-controller centre.

### 5. MAVLink Communication

The custom bridge, `vio_mavlink_bridge_modified.py`, uses ROS 2 and `pymavlink` to transmit external odometry to the Orange Cube flight controller.

**Bridge workflow**

1. Establish a serial connection to the flight controller.
2. Wait for an ArduPilot heartbeat.
3. Subscribe to `/visual_slam/tracking/odometry`.
4. Extract the pose, orientation, velocity, angular velocity, and covariance fields.
5. Apply the configured coordinate transformations.
6. Transmit the transformed data using MAVLink `ODOMETRY` messages.
7. Print periodic diagnostics showing the outgoing message rate and transformed state.

**Serial configuration**

The supplied script uses a USB serial device path and a baud rate of 115200. The device path must be updated to match the user's Jetson installation.

Check available serial devices with:

```bash
ls -l /dev/serial/by-id/
```

### 6. Running the MAVLink Bridge

Open Terminal 2 and start a second container using the same Docker image and workspace mount shown above.

Source ROS 2:

```bash
source /opt/ros/jazzy/setup.sh
```

Navigate to the directory containing `vio_mavlink_bridge_modified.py`. For example:

```bash
cd /workspaces/isaac_ros-dev
```

Run the bridge:

```bash
MAVLINK20=1 python3 vio_mavlink_bridge_modified.py
```

Keep Terminal 1 running so that Isaac ROS Visual SLAM continues publishing odometry while Terminal 2 runs the bridge.

The bridge should report the heartbeat connection and periodically print the outgoing odometry rate and transformed state.

### 7. ArduPilot External-Navigation Integration

The Orange Cube must be configured to accept and use the incoming external-navigation measurements.

Relevant setup includes:

- Verifying the serial port and baud rate.
- Configuring the appropriate ArduPilot EKF source parameters.
- Verifying the accepted position, velocity, and yaw sources.
- Checking coordinate frames, timestamps, and covariance values.
- Monitoring estimator health and position estimates through the ground-control station.

ArduPilot configuration depends on the firmware version and EKF setup.

Reference: https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html

Successful MAVLink transmission does not, by itself, confirm that the EKF is using the external odometry or that the vehicle is ready for flight.

## Experimental Validation

### 1. Hover Evaluation

Hover performance was evaluated during a continuous 10-minute flight using Vicon motion-capture measurements as an external reference.

The evaluation documentation will cover:

- Test conditions and commanded hover position.
- Time synchronization and coordinate alignment.
- Position-estimation error relative to Vicon.
- Hover-position error relative to the commanded position.
- Error metrics and supporting plots.

### 2. Altitude Tracking

An external ToF sensor reduced altitude-tracking error by 20% relative to the baseline configuration under the tested conditions.

The supporting documentation will identify:

- Baseline altitude-estimation configuration.
- ToF sensor integration method.
- Error metric used.
- Baseline and updated error values.
- Test conditions and supporting data.

## Flight-Test and Safety Procedure

The integration should be validated progressively before flight:

1. **Bench testing:** Run Visual SLAM and the MAVLink bridge with the propellers removed.
2. **Odometry validation:** Verify position, orientation, velocity, frame transformations, and tracking stability.
3. **Communication validation:** Confirm that MAVLink `ODOMETRY` messages are reaching the flight controller.
4. **Estimator validation:** Confirm that ArduPilot accepts the external-navigation measurements and reports a healthy estimator.
5. **Controlled flight testing:** Proceed only after the preceding checks pass, with an appropriate recovery mode and suitable test environment.

Incorrect frame transformations, camera extrinsics, or estimator configuration can lead to loss of position control.

## Ongoing Work

Adaptive control is under development to support drone-based non-destructive testing of pipelines.

The intended application is pipeline inspection, with ongoing work focused on maintaining stable flight and controlled physical contact during inspection operations.

This repository distinguishes completed positioning and flight-evaluation work from ongoing adaptive-control development.

## Repository Contents

```text
.
├── src/
│   └── vio_mavlink_bridge.py
├── config/
│   └── ardupilot_parameters.param

```

The directory structure above is a suggested organization; include only files that actually exist in the repository.

## References and Acknowledgements

- NVIDIA Isaac ROS Visual SLAM: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
- ArduPilot: https://github.com/ArduPilot/ardupilot
- ArduPilot Non-GPS Navigation: https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html

Upstream software remains the work of its respective contributors. Custom integration code and experimental results should be distinguished from upstream functionality.
