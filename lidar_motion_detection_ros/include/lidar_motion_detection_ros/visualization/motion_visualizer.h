#ifndef LIDAR_MOTION_DETECTION_ROS_VISUALIZATION_MOTION_VISUALIZER_H_
#define LIDAR_MOTION_DETECTION_ROS_VISUALIZATION_MOTION_VISUALIZER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <visualization_msgs/Marker.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/utils/color_maps.h>
#include <voxblox_ros/tsdf_server.h>

#include "lidar_motion_detection/3rd_party/config_utilities.hpp"
#include "lidar_motion_detection/common/types.h"

namespace motion_detection {

class MotionVisualizer {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    std::string global_frame_name = "map";

    // Set RGBA colors in [0, 1] if wanted.
    std::vector<float> static_point_color = {0.f, 0.f, 0.f, 1.f};
    std::vector<float> dynamic_point_color = {1.f, 0.f, 0.5f, 1.f};
    std::vector<float> sensor_color = {1.f, 0.f, 0.f, 1.f};

    // Scales of pointcloud [m].
    float static_point_scale = 0.1f;
    float dynamic_point_scale = 0.1f;
    float sensor_scale = 0.3f;

    // Number of colors for the a full color wheel revolution.
    int color_wheel_num_colors = 20;

    // True: every cluster and object has a color, False: just mark them as
    // dynamic.
    bool color_clusters = true;

    Config() { setConfigName("MotionVisualizer"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // Setup.
  MotionVisualizer(ros::NodeHandle nh,
                   std::shared_ptr<voxblox::TsdfServer> tsdf_server);

  void setupRos();

  // Visualization.
  void visualizeAll(const Cloud& cloud, const CloudInfo& cloud_info,
                    const Clusters& clusters);
  void visualizeLidarPose(const CloudInfo& cloud_info);
  void visualizeLidarPoints(const Cloud& cloud);
  void visualizePointDetections(const Cloud& cloud,
                                const CloudInfo& cloud_info);
  void visualizeClusterDetections(const Cloud& cloud,
                                  const CloudInfo& cloud_info,
                                  const Clusters& clusters);
  void visualizeMesh();

 private:
  const Config config_;
  voxblox::ExponentialOffsetIdColorMap color_map_;
  ros::NodeHandle nh_;
  std::shared_ptr<voxblox::TsdfServer> tsdf_server_;
  std::shared_ptr<voxblox::MeshIntegrator<voxblox::TsdfVoxel>> mesh_integrator_;
  std::shared_ptr<voxblox::MeshLayer> mesh_layer_;

  // Publishers.
  ros::Publisher sensor_pose_pub_;
  ros::Publisher sensor_points_pub_;
  ros::Publisher detection_points_pub_;
  ros::Publisher detection_points_comp_pub_;
  ros::Publisher detection_cluster_pub_;
  ros::Publisher detection_cluster_comp_pub_;
  ros::Publisher detection_object_pub_;
  ros::Publisher detection_object_comp_pub_;
  ros::Publisher ever_free_pub_;
  ros::Publisher never_free_pub_;
  ros::Publisher mesh_pub_;
};

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_ROS_VISUALIZATION_MOTION_VISUALIZER_H_
