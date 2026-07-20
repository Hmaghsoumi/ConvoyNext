# ConvoyNext

Official code repository for the paper:

**ConvoyNext: A Scalable Testbed Platform for Cooperative Autonomous Vehicle Systems**

Hossein Maghsoumi and Yaser Fallah  
2025 IEEE 102nd Vehicular Technology Conference (VTC2025-Fall)

## Overview

ConvoyNext is a modular and extensible research platform for the
real-world evaluation of cooperative autonomous vehicle systems.

The platform supports:

- UDP-based inter-vehicle communication
- J2735-inspired Basic Safety Messages
- Configurable communication topologies
- Controlled packet-loss experiments
- GPS-to-local coordinate transformation
- Straight and curved trajectory modeling
- Stanley-based lateral control
- P, PI, PD, and PID speed controllers
- Optimization-based cooperative motion generation
- Experimental data logging

## Repository Status

This repository currently provides the core platform-independent
control and communication implementation used in ConvoyNext.

The supplied Python module is not a standalone ROS node. A
platform-specific ROS/MAVROS interface is required to connect the
core implementation to vehicle sensors and actuators.

## Main File

`src/convoynext/control.py`

The file includes:

- `ROSArgs`: runtime and controller configuration
- `BasicSafetyMessage`: shared vehicle-state message representation
- `Control`: communication, trajectory analysis, controller, and
  data-logging functionality

## Requirements

- Python 3.9 or later
- NumPy
- SciPy
- ROS and MAVROS for physical vehicle deployment

Install the Python dependencies with:

```bash
pip install -r requirements.txt
