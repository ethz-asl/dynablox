#ifndef LIDAR_MOTION_DETECTION_ROS_VISUALIZATION_MOTION_VISUALIZER_H_
#define LIDAR_MOTION_DETECTION_ROS_VISUALIZATION_MOTION_VISUALIZER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <visualization_msgs/Marker.h>
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

    // Scales of pointcloud [m].
    float static_point_scale = 0.1f;
    float dynamic_point_scale = 0.1f;

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
  void visualizeLidarPoints(const Cloud& cloud);
  void visualizePointDetections(const Cloud& cloud,
                                const CloudInfo& cloud_info);
  void visualizeClusterDetections(const Cloud& cloud,
                                  const CloudInfo& cloud_info,
                                  const Clusters& clusters);

  // Old.
  // struct visTypePointcloud {
  //   ros::Publisher publisher;
  //   pcl::PointCloud<pcl::PointXYZRGB> cloud;
  // };

  // typedef struct visTypeMarker {
  //   ros::Publisher publisher;
  //   visualization_msgs::MarkerArray marker_arr;
  // };
  // void initializePointcloudPublishers();
  // void setupColors();
  // void setAllCloudsToVisualize(
  //     const pcl::PointCloud<pcl::PointXYZ>& processed_pcl);
  // void publishAll();
  // void setEverFreeLevelDetectionsCloud(
  //     const pcl::PointCloud<pcl::PointXYZ>& processed_pcl);
  // void setClusterLevelDetectionsCloud();
  // void setObjectLevelDetectionsCloud();
  // void setGroundTruthDetectionsCloud(
  //     const pcl::PointCloud<pcl::PointXYZ>& processed_pcl);
  // void setLidarPointcloudWithoutDynamicPoints(
  //     const pcl::PointCloud<pcl::PointXYZ>& processed_pcl);
  // void setNeverFreeVoxelsCloud();
  // void setEverFreeSliceCloud();
  // void initNewPointcloudToVisualize(const std::string& topic_name,
  //                                   const std::string& frame);
  // void initNewPointcloudToVisualize(const std::string& topic_name);
  // void initNewMarkerArrayToVisualize(const std::string& topic_name);
  // ros::Publisher* getPointcloudPublisher(const std::string& topic_name);
  // ros::Publisher* getMarkerPublisher(const std::string& topic_name);
  // pcl::PointCloud<pcl::PointXYZRGB>* getPointcloud(
  //     const std::string& topic_name);
  // void setPointcloud(const std::string& topic_name,
  //                    const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
  //                    const voxblox::Color& color);
  // void clearPointcloud(const std::string& topic_name);
  // void clearMarkerArray(const std::string& topic_name);
  // void publishPointcloud(const std::string& topic_name);
  // void publishMarkerArray(const std::string& topic_name);
  // void addMarkerToArray(const std::string& topic_name,
  //                       visualization_msgs::Marker* marker);
  // void addPointToPointcloud(const std::string& topic_name,
  //                           const pcl::PointXYZ& point,
  //                           const voxblox::Color& color);
  // void initMaxRaylengthVisualizer(const float& max_raylength,
  //                                 const voxblox::Color& color,
  //                                 const std::string& frame,
  //                                 const int num_points = 100);
  // void publishMaxRaylengthCircle();

 private:
  const Config config_;
  voxblox::ExponentialOffsetIdColorMap color_map_;
  ros::NodeHandle nh_;
  std::shared_ptr<voxblox::TsdfServer> tsdf_server_;

  // Publishers.
  ros::Publisher sensor_points_pub_;
  ros::Publisher detection_points_pub_;
  ros::Publisher detection_points_comp_pub_;
  ros::Publisher detection_cluster_pub_;
  ros::Publisher detection_cluster_comp_pub_;
  ros::Publisher detection_object_pub_;
  ros::Publisher detection_object_comp_pub_;
  ros::Publisher ever_free_pub_;
  ros::Publisher never_free_pub_;

  // Old.
  // CloudInfo* point_classifications_ptr_;
  // std::vector<Cluster>* current_clusters_ptr_;

  // std::string world_frame_;
  // std::string sensor_frame_;

  // MaxRaylengthIndicator max_raylength_circle_;
  // ros::Publisher max_raylength_pub_;

  // std::unordered_map<std::string, visTypePointcloud>
  // pointcloud_vis_container_; std::unordered_map<std::string, visTypeMarker>
  // marker_vis_container_;

  // voxblox::Color point_and_cluster_candidate_color_;
  // voxblox::Color never_free_color_;
  // voxblox::Color lidar_point_color_;
  // voxblox::Color sensor_origin_color_;
};

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_ROS_VISUALIZATION_MOTION_VISUALIZER_H_
