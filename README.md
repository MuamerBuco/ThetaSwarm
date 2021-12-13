# OpenSwarm Application Software v1.0

OpenSwarm is an Open Source Robotic Hardware/Software research platform. This application, together with the OpenSwarm Firmware, make up the software part. The system requires a USB Camera that will observe the intended play-space, and forward the captured frames to the application. This is required because the system pose estimation feedback is based on the OpenCV Aruco algorithm and the corresponding markers attached to the robots. This application software is doing most of the heavy lifting, from marker detection and pose estimation to robot initialization, communication, kinematics, trajectory planning and interfacing. The hardware is an arbitrary number of very low-cost and easy-to-build autonomous mobile robots that are fully 3D printable. The Robot STLs and circuit layouts are available at: """".

This system is meant to offer fully abstracted access to hardware, to be leveraged by custom control algorithms and learning systems.


# Still Under Development....

![robotImage1](https://github.com/MuamerBuco/ThetaSwarm/blob/master/images/IMG_6040_00.png?raw=true =250x250)
 
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

- Multi-robot support
- Swarm level control
- Low-Level command interface
- Custom camera settings
- Point-to-Point control
- Trajectory generation

## Coming Soon

- Trajectory splining
- Dynamic trajectory generation
- Obstacle avoidance
- Automated scene setup
- Built-in camera calibration
- Python <-> C++ piping
- AutoTune PID
- ...

