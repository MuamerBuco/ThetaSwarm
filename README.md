# ThetaSwarm Application Software v1.0

ThetaSwarm is an Open Source Robotic Hardware/Software research platform. 

This multi-threaded application, together with the ThetaSwarm Firmware, make up the software part. The system requires a USB Camera that will observe the intended workspace, and forward the captured frames to the application. This is required because the system pose estimation feedback is based on the OpenCV Aruco algorithm and requires the corresponding markers attached to the robots. This application software is doing most of the heavy lifting, from marker detection and pose estimation to robot initialization, communication, kinematics, autonomous navigation and interfacing. 

The hardware is an arbitrary number of very low-cost and easy-to-build autonomous mobile robots controlled via WiFi, that are fully 3D printable. The Robot STL/3MFs and circuit layouts are available at: https://1drv.ms/u/s!Ake4VfEvlOEKshU9qw6VmYhgfrLv?e=WLa5Gi.

This system is meant to offer fully abstracted access to hardware, to be leveraged by custom control algorithms and learning systems.


# Still Under Development....

<img src="https://github.com/MuamerBuco/ThetaSwarm/blob/master/images/IMG_6104_00.png" alt="robotImage1" width="450" height="340" align="left">
<img src="https://github.com/MuamerBuco/ThetaSwarm/blob/master/images/IMG_6112_00Cut.png" alt="robotImage2" width="300" height="340">
 
## Requirements

- CMake (version 3.16 or higher)
- OpenCV (version 4.0 or higher)
- OMPL (version 1.5 or higher)
- BOOST (version 1.58 or higher)
- Eigen (version 3.3 or higher)


## Build The Project

```
mkdir -p build/Release
cd build/Release
cmake ../..

# next step is optional
make -j 4 # replace "4" with the number of cores on your machine
```

## Current Features

- Autonomous navigation
- Trajectory generation
- Wide selection of path planning algorithms
- Multi-robot support
- Swarm level control
- Dead reckoning
- Speed/Accel metrics
- Cybernetic control
- Custom camera settings
- Direct 2DOF effector control
- LED signaling and effects

## Coming Soon

- Reinforcement Learning for motion control
- Trajectory splining
- Dynamic trajectory generation
- Obstacle avoidance
- Automated scene setup
- Built-in camera calibration
- Piping
- AutoTune PID
- ...
