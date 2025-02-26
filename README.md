# ESDF_MAP 

[English](#esdf_map-1)

实时雷达点云局部ESDF建图系统 | Real-time Local ESDF Mapping with Radar Point Clouds

## 📖 概述 | Overview

基于ROS与Voxblox实现动态局部欧几里得符号距离场（ESDF）的实时构建与更新，专为大型场景下无法维护全局地图的移动机器人设计。  
Real-time local Euclidean Signed Distance Field (ESDF) mapping system using ROS and Voxblox, specifically designed for mobile robots operating in large-scale environments where global map maintenance is impractical.

![Build Status](https://img.shields.io/badge/build-passing-brightgreen) ![ROS](https://img.shields.io/badge/ROS-Noetic-blue) ![License](https://img.shields.io/badge/license-BSD--3-green)

## 🛠️ 依赖安装 | Dependencies Installation

### Voxblox 安装
```bash
# 官方推荐安装方式（Ubuntu 20.04 + ROS Noetic）
# Official installation (Ubuntu 20.04 + ROS Noetic)
sudo apt-get install ros-noetic-voxblox ros-noetic-voxblox-msgs ros-noetic-voxblox-rviz-plugin
```
或遵循官方文档：<https://voxblox.readthedocs.io/en/latest/pages/Installation.html>

## 💻 安装 | Installation

1. 创建工作空间
```bash
mkdir -p voxblox_ws/src && cd voxblox_ws/src
```

2. 克隆本仓库
```bash
git clone https://github.com/JaimeEden/ESDF_MAP.git
```

3. 编译包
```bash
cd .. && catkin build radar_esdf
```

## 🚀 运行 | Running

```bash
# 初始化环境
source devel/setup.bash

# 启动建图节点（需提前连接雷达设备）
roslaunch radar_esdf radar_esdf_mapper.launch \
   sensor_model:=vlp16 \           # 雷达型号选择(vlp16/hdl32)
   update_rate:=15 \               # ESDF更新频率(Hz)
   map_radius:=15.0                # 局部地图半径(m)
```

## 🗺️ 可视化
使用Rviz查看实时ESDF地图：  
```bash
roslaunch radar_esdf visualization.launch
```
![ESDF Visualization](docs/esdf_demo.gif)

---

# ESDF_MAP 

## 📖 Overview

Real-time local Euclidean Signed Distance Field (ESDF) mapping system using ROS and Voxblox, specifically designed for mobile robots operating in large-scale environments where global map maintenance is impractical.

![Build Status](https://img.shields.io/badge/build-passing-brightgreen) ![ROS](https://img.shields.io/badge/ROS-Noetic-blue) ![License](https://img.shields.io/badge/license-BSD--3-green)

## 🛠️ Dependencies

### Voxblox Installation
```bash
# Recommended for Ubuntu 20.04 + ROS Noetic
sudo apt-get install ros-noetic-voxblox ros-noetic-voxblox-msgs ros-noetic-voxblox-rviz-plugin
```
Or follow official docs: <https://voxblox.readthedocs.io/en/latest/pages/Installation.html>

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
roslaunch radar_esdf radar_esdf_mapper.launch \
   sensor_model:=vlp16 \           # LiDAR model (vlp16/hdl32)
   update_rate:=15 \               # ESDF update rate(Hz)
   map_radius:=15.0                # Local map radius(m)
```

## 🗺️ Visualization
View real-time ESDF in Rviz:  
```bash
roslaunch radar_esdf visualization.launch
```
![ESDF Visualization](docs/esdf_demo.gif)
