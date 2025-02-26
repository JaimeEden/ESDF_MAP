# ESDF_MAP 

实时雷达点云局部ESDF建图系统 | Real-time Local ESDF Mapping with Radar Point Clouds

## 📖 Overview

Real-time local Euclidean Signed Distance Field (ESDF) mapping system using ROS and Voxblox, specifically designed for mobile robots operating in large-scale environments where global map maintenance is impractical.

![Build Status](https://img.shields.io/badge/build-passing-brightgreen) ![ROS](https://img.shields.io/badge/ROS-Noetic-blue) ![License](https://img.shields.io/badge/license-BSD--3-green)

## 🛠️ Dependencies

### Voxblox Installation

Follow official docs: <https://voxblox.readthedocs.io/en/latest/pages/Installation.html>

## 💻 Installation

1. Create workspace
```bash
mkdir -p voxblox_ws/src && cd voxblox_ws/src
```

2. Clone repository
```bash
git clone https://github.com/JaimeEden/ESDF_MAP.git
```

3. Build package
```bash
cd .. && catkin build radar_esdf
```

## 🚀 Running

```bash
# Initialize environment
source devel/setup.bash

# Launch mapping node (require radar connection)
roslaunch radar_esdf radar_esdf_mapper.launch 
```
