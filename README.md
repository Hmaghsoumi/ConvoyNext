# ConvoyNext

Official code repository for the paper:

**ConvoyNext: A Scalable Testbed Platform for Cooperative Autonomous Vehicle Systems**

**Authors:** Hossein Maghsoumi and Yaser Fallah  
**Conference:** 2025 IEEE 102nd Vehicular Technology Conference (VTC2025-Fall)  
**DOI:** 10.1109/VTC2025-Fall65116.2025.11310499

## Overview

ConvoyNext is a research testbed for real-world cooperative autonomous vehicle experiments. The current implementation supports inter-vehicle state exchange, communication-loss emulation, cooperative trajectory analysis, longitudinal and lateral control, ROS 2/MAVROS integration, and experiment logging.

The main platform logic is implemented in `src/crossplatform.py`.

## Repository Structure

```text
ConvoyNext/
├── README.md
├── LICENSE
├── CITATION.cff
├── requirements.txt
├── src/
│   ├── crossplatform.py
│   ├── control_ros2.py
│   ├── platooning.py
│   └── beacon_broadcast.py
└── tracks/
    └── garage_c_loop_big.json
```

## Main Components

### `src/crossplatform.py`

The main implementation contains:

- Runtime and controller configuration through `ROSArgs`
- Basic Safety Message representation
- UDP multicast communication
- Configurable packet-drop emulation
- Sensor-state handling
- GPS-to-local coordinate transformation
- Straight and curved trajectory modeling
- Cooperative target-motion optimization
- Stanley lateral control
- P, I, PI, PD, and PID speed control
- Experimental data logging

### `src/control_ros2.py`

Provides the ROS 2 and MAVROS interface, including:

- Sensor subscribers
- Velocity command publishing
- Vehicle arming and disarming
- Mission-state handling
- Periodic communication and control callbacks

### `src/platooning.py`

Command-line entry point for running a follower vehicle.

### `src/beacon_broadcast.py`

Generates a virtual leader trajectory from a track JSON file and broadcasts vehicle-state messages over UDP multicast.

### `tracks/garage_c_loop_big.json`

Defines the straight segments, turns, speeds, and reference center of an example test track.

## Requirements

The Python dependencies are listed in `requirements.txt`.

Install them with:

```bash
python3 -m pip install -r requirements.txt
```

ROS 2, MAVROS, PX4-related message packages, and the vehicle-specific runtime must be installed separately through the appropriate ROS distribution and operating-system package manager.

## Running the Virtual Leader

From the repository root:

```bash
python3 src/beacon_broadcast.py \
  --track_name tracks/garage_c_loop_big.json \
  --broadcast_interval 0.1 \
  --drop_rate 0.0
```

The default multicast configuration used by the current code is:

- Multicast group: `224.0.0.1`
- UDP port: `5004`

Make sure all participating machines are connected to the same network and that multicast traffic is permitted.

## Running a Follower Vehicle

A typical ROS 2 command has the following form:

```bash
python3 src/platooning.py ros2 \
  --track_path tracks/garage_c_loop_big.json \
  --car_number 1 \
  --heading_con_type Stanley_Curve \
  --speed_con_type PID \
  --follow_distance 1.0 \
  --drop_rate 0.0
```

The command assumes that ROS 2, MAVROS, the vehicle flight controller, and the required topics and services are already available.

## Implementation Structure

`src/crossplatform.py` contains the core platform-independent implementation of ConvoyNext, including communication, coordinate transformation, cooperative trajectory analysis, control, and data logging.

The remaining Python files provide the ROS 2/MAVROS interface, command-line execution, and virtual-leader functionality.

## Safety Notice

This repository controls physical autonomous vehicles. Test all changes in simulation or with the vehicle elevated and wheels unloaded before conducting ground experiments. Use an independent emergency stop, maintain a clear test area, and follow all institutional and equipment-specific safety procedures.

## Citation

Please cite the following paper when using this repository:

```bibtex
@inproceedings{maghsoumi2025convoynext,
  title     = {ConvoyNext: A Scalable Testbed Platform for Cooperative Autonomous Vehicle Systems},
  author    = {Maghsoumi, Hossein and Fallah, Yaser},
  booktitle = {2025 IEEE 102nd Vehicular Technology Conference (VTC2025-Fall)},
  year      = {2025},
  publisher = {IEEE},
  doi       = {10.1109/VTC2025-Fall65116.2025.11310499}
}
```
