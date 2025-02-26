# ESDF_MAP
This repository implements real-time local ESDF (Euclidean Signed Distance Field) mapping with dynamic updates using radar point clouds through ROS and Voxblox, specifically designed for mobile robots operating in large-scale environments where global map accumulation is impractical.

#依赖安装

按照https://voxblox.readthedocs.io/en/latest/pages/Installation.html的教程安装voxblox库。

#本仓库安装

cd voxblox_ws/src

git clone 

cd ..

catkin build radar_esdf

#运行

source devel/setup.bash

roslaunch radar_esdf radar_esdf_mapper.launch
