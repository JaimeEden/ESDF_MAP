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

class RadarEsdfMapper {
public:
  RadarEsdfMapper() 
    : nh_("~"),
      esdf_server_(nh_,nh_),
      tf_listener_(tf_buffer_) 
  {
    // 初始化参数
    max_range_ = 1500.0;  // 局部地图最大范围
    max_distance_ = 10.0;  // ESDF最大距离
    voxel_size_ = 3.0;  // 体素大小

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

    integrator_config.voxel_carving_enabled = true;
    integrator_config.max_ray_length_m = max_range_;

    integrator_config.default_truncation_distance = max_distance_;
    tsdf_integrator_ = std::make_shared<voxblox::SimpleTsdfIntegrator>(
        integrator_config, 
        esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()
    );    

    if (publish_esdf_) {
      //esdf_server_.setPublishEsdf(true);
      esdf_server_.setPublishSlices(true);
      esdf_server_.setSliceLevel(2.0);  // 发布高度0.5米的2D切片
    }

    // 订阅雷达点云 /points_raw /PointCloudDetection
    pointcloud_sub_ = nh_.subscribe("/PointCloudDetection", 10,
                                   &RadarEsdfMapper::pointcloudCallback, this);
    
    // 发布ESDF点云
    esdf_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/esdf_cloud", 1);

    // 初始化滤波器
    voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  }

private:
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    // 设置全局坐标系
    //esdf_server_.setGlobalFrame(msg->header.frame_id);

    esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->removeAllBlocks();
    esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr()->removeAllBlocks();

    // 1. 转换点云到世界坐标系

    // 直接使用原始点云数据
    sensor_msgs::PointCloud2 cloud_world = *msg;


    // 2. 转换为PCL点云并进行滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_world, *pcl_cloud);

    for (const auto& point : *pcl_cloud)
    {
        ROS_INFO("Point: x = %f, y = %f, z = %f", point.x, point.y, point.z);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter_.setInputCloud(pcl_cloud);
    voxel_filter_.filter(*filtered_cloud);

    // 3. 转换为voxblox点云格式
    voxblox::Pointcloud voxblox_points;
    voxblox::Colors voxblox_colors;
    for (const auto& p : filtered_cloud->points) {
      if (std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) > max_range_) continue;
      voxblox_points.emplace_back(p.x, p.y, p.z);
      voxblox_colors.push_back(voxblox::Color::Gray());
    }


    // 5. 将点云插入TSDF地图
    // voxblox::TsdfIntegratorBase::Config integrator_config;
    // integrator_config.voxel_carving_enabled = true;
    // integrator_config.max_ray_length_m = max_range_;
    // integrator_config.default_truncation_distance = max_distance_;
    
    // auto tsdf_integrator = std::make_shared<voxblox::SimpleTsdfIntegrator>(
    //         integrator_config, 
    //         esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr());

    // auto tsdf_integrator = std::make_shared<voxblox::MergedTsdfIntegrator>(
    //     integrator_config, 
    //     esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr());

    tsdf_integrator_->integratePointCloud(voxblox::Transformation(),
                                        voxblox_points,
                                        voxblox_colors);

    // 6. 更新ESDF地图
    esdf_server_.updateEsdf();

    esdf_server_.publishPointclouds();
    esdf_server_.publishSlices();

    // 7. 生成并发布ESDF点云
    //generateEsdfPointcloud();
  }

//   void generateEsdfPointcloud() 
//   {
//     pcl::PointCloud<pcl::PointXYZI> esdf_cloud;
//     const auto& esdf_layer = esdf_server_.getEsdfMapPtr()->getEsdfLayer();

//     // 检查 ESDF 层是否有效
//     if (!esdf_server_.getEsdfMapPtr() || esdf_layer.getNumberOfAllocatedBlocks() == 0) {
//         ROS_ERROR("ESDF layer is not initialized or empty!");
//         return;
//     }

//     voxblox::BlockIndexList block_indices;
//     esdf_layer.getAllAllocatedBlocks(&block_indices);

//     for (const auto& block_index : block_indices) {
//         const auto block = esdf_layer.getBlockPtrByIndex(block_index);
//         if (!block) {
//             ROS_WARN("Block at index (%ld, %ld, %ld) is null!", block_index.x(), block_index.y(), block_index.z());
//             continue;
//         }

//         for (int x = 0; x < block->num_voxels(); ++x) {
//             for (int y = 0; y < block->num_voxels(); ++y) {
//                 for (int z = 0; z < block->num_voxels(); ++z) {
//                     // 检查索引范围
//                     if (x < 0 || x >= block->num_voxels() ||
//                         y < 0 || y >= block->num_voxels() ||
//                         z < 0 || z >= block->num_voxels()) {
//                         ROS_ERROR("Voxel index out of bounds: (%d, %d, %d)", x, y, z);
//                         continue;
//                     }

//                     const auto& voxel = block->getVoxelByVoxelIndex(voxblox::VoxelIndex(x, y, z));
//                     if (!voxel.observed) {
//                         continue;  // 跳过未观测的体素
//                     }

//                     if (std::abs(voxel.distance) < max_distance_) 
//                     {
//                         const auto position = block->computeCoordinatesFromVoxelIndex(
//                             voxblox::VoxelIndex(x, y, z));

//                         // 检查坐标是否有效
//                         if (std::isnan(position.x()) || std::isnan(position.y()) || std::isnan(position.z())) {
//                             ROS_ERROR("Invalid coordinates computed for voxel index (%d, %d, %d)", x, y, z);
//                             continue;
//                         }

//                         pcl::PointXYZI point;
//                         point.x = position.x();
//                         point.y = position.y();
//                         point.z = position.z();
//                         point.intensity = voxel.distance;
//                         esdf_cloud.push_back(point);
//                     }
//                     else
//                     {
//                       continue;
//                     }
//                 }
//             }
//         }
//     }

//     // 检查发布者是否有效
//     if (!esdf_pub_) {
//         ROS_ERROR("ESDF point cloud publisher is not initialized!");
//         return;
//     }

//     // 转换为ROS消息并发布
//     sensor_msgs::PointCloud2 output;
//     pcl::toROSMsg(esdf_cloud, output);
//     output.header.frame_id = "radar";
//     output.header.stamp = ros::Time::now();
//     esdf_pub_.publish(output);
// }

  

  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher esdf_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  voxblox::EsdfServer esdf_server_;
  std::shared_ptr<voxblox::SimpleTsdfIntegrator> tsdf_integrator_;
  voxblox::TsdfIntegratorBase::Config integrator_config;
  geometry_msgs::TransformStamped transform;
  bool publish_esdf_;
  bool publish_slices_;
  
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  float max_range_;      // 局部地图最大范围
  float max_distance_;   // ESDF最大距离
  float voxel_size_;     // 体素大小
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "radar_esdf_mapper");
  RadarEsdfMapper mapper;
  ros::spin();
  return 0;
}

// void generateEsdfPointcloud() {
  //   pcl::PointCloud<pcl::PointXYZI> esdf_cloud;
  //   const auto& esdf_layer = esdf_server_.getEsdfMapPtr()->getEsdfLayer();

  //   voxblox::BlockIndexList block_indices;
  //   esdf_layer.getAllAllocatedBlocks(&block_indices);

  //   for (const auto& block_index : block_indices) {
  //     const auto block = esdf_layer.getBlockPtrByIndex(block_index);
  //     if (!block) continue;

  //     for (int x = 0; x < block->num_voxels(); ++x) {
  //       for (int y = 0; y < block->num_voxels(); ++y) {
  //         for (int z = 0; z < block->num_voxels(); ++z) {
  //           const auto& voxel = block->getVoxelByVoxelIndex(voxblox::VoxelIndex(x, y, z));   // ?voxelIndex
  //           if (voxel.observed && std::abs(voxel.distance) < max_distance_) {
  //             const auto position = block->computeCoordinatesFromVoxelIndex(
  //                 voxblox::VoxelIndex(x, y, z));
              
  //             pcl::PointXYZI point;
  //             point.x = position.x();
  //             point.y = position.y();
  //             point.z = position.z();
  //             point.intensity = voxel.distance;
  //             esdf_cloud.push_back(point);
  //           }
  //         }
  //       }
  //     }
  //   }

  //   // 转换为ROS消息并发布
  //   sensor_msgs::PointCloud2 output;
  //   pcl::toROSMsg(esdf_cloud, output);
  //   output.header.frame_id = "radar";
  //   output.header.stamp = ros::Time::now();
  //   esdf_pub_.publish(output);
  // }