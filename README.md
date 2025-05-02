# Luna Robotics – NASA Lunabotics Challenge

A ROS 2-based autonomy stack for the 2025 NASA Lunabotics Challenge, featuring depth‐based object detection, costmap‐driven path planning, and real‐time localization on an Intel RealSense D455.

## 📋 Table of Contents

1. [Project Overview](#project-overview)  
2. [Features](#features)  
3. [Tech Stack & Dependencies](#tech-stack--dependencies)  
4. [Getting Started](#getting-started)  
   - [Prerequisites](#prerequisites)  
   - [Installation](#installation)  
   - [Launching Nodes](#launching-nodes)  
5. [Directory Structure](#directory-structure)  
6. [Usage Examples](#usage-examples)  
7. [Contributing](#contributing)  
8. [License](#license)  
9. [Contact & Résumé Link](#contact--résumé-link)  

## Project Overview

Our rover autonomously detects boulders and craters using an Intel RealSense D455, filters obstacles via RANSAC and plane‐segmentation, and plans around them with Nav2’s SMAC planner.

## Features

- **Depth‐based obstacle detection**: ground removal, wall‐filtering, Euclidean clustering  
- **Localization**: custom TF broadcaster from external pose estimator  
- **Path planning**: global planning with Nav2 SMAC, regulated pure pursuit controller  
- **Real‐time visualization**: RViz2 configurations for costmaps and point clouds  
- **Modular ROS 2 Python nodes**: easily extendable detection & filtering pipelines  

## 🧰 Tech Stack & Dependencies

- **Operating System**: Ubuntu 22.04 LTS  
- **ROS 2**: Humble Hawksbill  
- **Languages**: Python 3.10, C++17 (when applicable)  
- **ROS 2 Packages**:
  - `rclpy`, `nav2_smac_planner`, `nav2_controller`, `tf2_ros`, `sensor_msgs`
- **Third-party Libraries**:
  - `pyrealsense2` (Intel RealSense SDK v2.55.1)
  - `OpenCV` (≥ 4.5)
  - `numpy`, `scipy`
  - `pcl` via `python-pcl` or ROS wrappers
  - `transforms3d`
- **Build Tools**: `colcon`, `ament_cmake`, `ament_python`
