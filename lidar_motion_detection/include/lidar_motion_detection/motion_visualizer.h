#ifndef LIDAR_MOTION_DETECTION_MOTION_VISUALIZER_H_
#define LIDAR_MOTION_DETECTION_MOTION_VISUALIZER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <voxblox_ros/tsdf_server.h>

#include "lidar_motion_detection/common/types.h"
#include "lidar_motion_detection/visualization_utils.h"

namespace motion_detection {

class MotionVisualizer {
 public:
  typedef struct visTypePointcloud {
    ros::Publisher publisher;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
  } visTypePcl;

  typedef struct visTypeMarker {
    ros::Publisher publisher;
    visualization_msgs::MarkerArray marker_arr;
  } visTypeMarker;

  MotionVisualizer(const ros::NodeHandle& nh_private,
                   PointInfoCollection* point_clfs,
                   std::vector<Cluster>* current_clusters,
                   std::shared_ptr<voxblox::TsdfMap> tsdf_map);
  MotionVisualizer(const MotionVisualizer& visualizer) {}

  void initializePointcloudPublishers();
  void setupColors();
  void setAllCloudsToVisualize(
      const pcl::PointCloud<pcl::PointXYZ>& processed_pcl);
  void publishAll();

  void setEverFreeLevelDetectionsCloud(
      const pcl::PointCloud<pcl::PointXYZ>& processed_pcl);
  void setClusterLevelDetectionsCloud();
  void setObjectLevelDetectionsCloud();
  void setGroundTruthDetectionsCloud(
      const pcl::PointCloud<pcl::PointXYZ>& processed_pcl);

  void setLidarPointcloudWithoutDynamicPoints(
      const pcl::PointCloud<pcl::PointXYZ>& processed_pcl);

  void setNeverFreeVoxelsCloud();
  void setEverFreeSliceCloud();

  void initNewPointcloudToVisualize(const std::string& topic_name,
                                    const std::string& frame);
  void initNewPointcloudToVisualize(const std::string& topic_name);

  void initNewMarkerArrayToVisualize(const std::string& topic_name);

  ros::Publisher* getPointcloudPublisher(const std::string& topic_name);
  ros::Publisher* getMarkerPublisher(const std::string& topic_name);
  pcl::PointCloud<pcl::PointXYZRGB>* getPointcloud(
      const std::string& topic_name);

  void setPointcloud(const std::string& topic_name,
                     const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
                     const voxblox::Color& color);

  void clearPointcloud(const std::string& topic_name);
  void clearMarkerArray(const std::string& topic_name);

  void publishPointcloud(const std::string& topic_name);
  void publishMarkerArray(const std::string& topic_name);

  void addMarkerToArray(const std::string& topic_name,
                        visualization_msgs::Marker* marker);

  void addPointToPointcloud(const std::string& topic_name,
                            const pcl::PointXYZ& point,
                            const voxblox::Color& color);

  void initMaxRaylengthVisualizer(const float& max_raylength,
                                  const voxblox::Color& color,
                                  const std::string& frame,
                                  const int num_points = 100);

  void publishMaxRaylengthCircle();

 protected:
  std::shared_ptr<voxblox::TsdfMap> tsdf_map_;

 private:
  ros::NodeHandle nh_visualizer_;

  PointInfoCollection* point_classifications_ptr_;
  std::vector<Cluster>* current_clusters_ptr_;

  std::string world_frame_;
  std::string sensor_frame_;

  MaxRaylengthIndicator max_raylength_circle_;
  ros::Publisher max_raylength_pub_;

  std::unordered_map<std::string, visTypePointcloud> pointcloud_vis_container_;
  std::unordered_map<std::string, visTypeMarker> marker_vis_container_;

  voxblox::Color point_and_cluster_candidate_color_;
  voxblox::Color never_free_color_;
  voxblox::Color lidar_point_color_;
  voxblox::Color sensor_origin_color_;
};

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_MOTION_VISUALIZER_H_
