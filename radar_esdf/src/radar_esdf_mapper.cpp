#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox_ros/esdf_server.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <chrono>

voxblox::TsdfIntegratorBase::Config integrator_config;
voxblox::EsdfMap::Config esdf_config;
voxblox::TsdfMap::Config tsdf_config;

float max_range_ = 1500.0;      // 局部地图最大范围
float max_distance_ = 10.0;   // ESDF最大距离
float voxel_febianlv = 5.0; //地图分辨率体素
    
class RadarEsdfMapper {
public:
  RadarEsdfMapper() 
    : nh_("~"),
    esdf_server_(nh_, nh_, esdf_config, voxblox::EsdfIntegrator::Config(), tsdf_config, integrator_config, voxblox::MeshIntegratorConfig())
  {

    // 设置平移为 (0, 0, 0)
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;

    // 设置旋转为单位四元数 (0, 0, 0, 1)
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;


    // 配置ESDF服务器
    esdf_server_.setTraversabilityRadius(1.2);  //机器人或无人机半径
    esdf_server_.setClearSphere(true);
    esdf_server_.setEsdfMaxDistance(max_distance_);

    tsdf_integrator_ = std::make_shared<voxblox::FastTsdfIntegrator>(  //voxblox::SimpleTsdfIntegrator
        integrator_config, 
        esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()
    );    

   
      //esdf_server_.setPublishSlices(true);
      //esdf_server_.setSliceLevel(0.05);  // 发布高度0.5米的2D切片

    // 订阅雷达点云 /points_raw /PointCloudDetection
    pointcloud_sub_ = nh_.subscribe("/PointCloudDetection", 10,
                                   &RadarEsdfMapper::pointcloudCallback, this);
    

  }

private:
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->removeAllBlocks();
    esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr()->removeAllBlocks();

    // 1. 转换点云到世界坐标系
    sensor_msgs::PointCloud2 cloud_world = *msg;

    // 2. 转换为PCL点云并进行滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_world, *pcl_cloud);

    // 3. 转换为voxblox点云格式
    voxblox::Pointcloud voxblox_points;
    voxblox::Colors voxblox_colors;
    
    
    for (const auto& p : pcl_cloud->points) {
        voxblox_points.emplace_back(p.x, p.y, p.z);
        voxblox_colors.push_back(voxblox::Color::Gray());
    }
  
    // 4. TSDF积分
    tsdf_integrator_->integratePointCloud(
        voxblox::Transformation(), voxblox_points, voxblox_colors);
    

    // 5. ESDF更新（添加计时）
    // auto start_esdf_update = std::chrono::high_resolution_clock::now();
    esdf_server_.updateEsdf();
    // auto end_esdf_update = std::chrono::high_resolution_clock::now();
    // auto esdf_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
    //     end_esdf_update - start_esdf_update).count();
    // ROS_INFO_STREAM("[Performance] ESDF update time: " << esdf_duration << " ms");

    esdf_server_.publishPointclouds();
    esdf_server_.publishSlices();
}


  

  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_sub_;
  voxblox::EsdfServer esdf_server_;
  geometry_msgs::TransformStamped transform;
  std::shared_ptr<voxblox::FastTsdfIntegrator> tsdf_integrator_;  //SimpleTsdfIntegrator
  
};

void fuzhi()
{
    
    esdf_config.esdf_voxel_size = voxel_febianlv;  //设置分辨率
    
    tsdf_config.tsdf_voxel_size = voxel_febianlv;  //设置分辨率

    integrator_config.voxel_carving_enabled = true;
    integrator_config.max_ray_length_m = max_range_;

    integrator_config.default_truncation_distance = max_distance_;
    return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "radar_esdf_mapper");
  fuzhi();
  RadarEsdfMapper mapper;
  ros::spin();
  return 0;
}

