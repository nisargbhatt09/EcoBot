# EcoBot

# Mobile Manipulator Robot Simulation

A PyBullet-based simulation of a mobile manipulator robot (Husky + Kuka arm) performing autonomous pick-and-place operations with computer vision capabilities.

## Overview

This project implements a robotic system that combines:
- Mobile base navigation (Husky robot)
- Robotic manipulation (Kuka arm)
- Computer vision (YOLO object detection)
- Path planning (A* algorithm)

The robot autonomously navigates through a maze-like environment, identifies objects using computer vision, picks them up, and sorts them into designated trays.

## Components

### 1. GarbageCollector.py
Main execution file that integrates all components and implements:
- Robot control and coordination
- Computer vision integration
- Pick and place operations
- Real-time simulation management

### 2. AstarAlgorithm.py
Path planning implementation featuring:
- A* algorithm for optimal path finding
- Obstacle avoidance
- Path optimization
- Grid-based navigation system

### 3. husky_kuka_motion_control.py
Robot configuration and setup:
- Husky mobile base initialization
- Kuka arm mounting and configuration
- Environment setup
- Physics simulation parameters

### 4. plot.py
Performance visualization tools for:
- Navigation time analysis
- Grasp success rates
- Path length metrics
- Task execution timing

## Key Features

- Real-time object detection using YOLO
- Smooth path planning with A* algorithm
- Precise robotic manipulation
- End-effector camera feed
- Collision avoidance
- Performance metrics visualization

## Dependencies

- PyBullet
- NumPy
- OpenCV
- PyTorch (for YOLO)
- Matplotlib (for visualization)

## Installation

pip install pybullet numpy opencv-python torch matplotlib

## Usage

1. Run the main simulation:
python GarbageCollector.py


2. View performance metrics:
python plot.py


## Performance Metrics

The system tracks:
- Navigation time per path
- Grasp success rates for different objects
- Path lengths
- Task execution times
