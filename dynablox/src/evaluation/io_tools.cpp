#include "dynablox/evaluation/io_tools.h"

#include <filesystem>
#include <fstream>
#include <vector>

namespace dynablox {

bool saveCloudToCsv(const std::string& file_name, const Cloud& cloud,
                    const CloudInfo& cloud_info, const Clusters& clusters,
                    const int cloud_id) {
  // Initialize headers if required.
  std::fstream writefile;
  const bool file_existed = std::filesystem::exists(file_name);
  writefile.open(file_name, std::ios::app);
  if (!writefile.is_open()) {
    return false;
  }
  if (!file_existed) {
    writefile << "CloudNo,X,Y,Z,Distance,PointDynamic,ClusterDynamic,"
                 "ObjectDynamic,GTDynamic,ReadyForEvaluation,ClusterID\n";
  }

  // Add all new data to the database.
  size_t i = 0;
  for (const Point& point : cloud) {
    const PointInfo& info = cloud_info.points.at(i);
    int cluster_id = -1;
    if (info.cluster_level_dynamic) {
      for (const Cluster& cluster : clusters) {
        if (std::find(cluster.points.begin(), cluster.points.end(), i) !=
            cluster.points.end()) {
          cluster_id = cluster.id;
          break;
        }
      }
    }
    ++i;

    writefile << cloud_id << "," << point.x << "," << point.y << "," << point.z
              << "," << info.distance_to_sensor << ","
              << info.ever_free_level_dynamic << ","
              << info.cluster_level_dynamic << "," << info.object_level_dynamic
              << "," << info.ground_truth_dynamic << ","
              << info.ready_for_evaluation << "," << cluster_id << "\n";
  }
  writefile.close();
  return true;
}

bool loadCloudFromCsv(const std::string& file_name, std::vector<Cloud>& clouds,
                      std::vector<CloudInfo>& cloud_infos,
                      std::vector<Clusters>& clusters) {
  // Open file.
  std::ifstream readfile;
  readfile.open(file_name);
  if (!readfile.is_open()) {
    return false;
  }

  // Read data.
  std::string line;
  int cloud_counter = -2;
  std::unordered_map<int, size_t>
      cluster_id_to_index;  // [cluster_id] = position in clusters vector for
                            // this frame.
  while (getline(readfile, line)) {
    if (line.empty()) {
      continue;
    }
    if (cloud_counter == -2) {
      // Skip headers.
      cloud_counter++;
      continue;
    }
    std::istringstream iss(line);
    std::string linestream;
    size_t row_index = 0;
    Point* point;
    PointInfo* info;
    while (std::getline(iss, linestream, ',')) {
      switch (row_index) {
        case 0u: {
          const int cloud_id = std::stoi(linestream);
          if (cloud_id != cloud_counter) {
            // Initialize new cloud.
            clouds.push_back(Cloud());
            cloud_infos.push_back(CloudInfo());
            clusters.push_back(Clusters());
            cluster_id_to_index.clear();
            cloud_infos.back().has_labels = true;
            cloud_counter++;
          }
          clouds.back().push_back(Point());
          point = &clouds.back().back();
          cloud_infos.back().points.push_back(PointInfo());
          info = &cloud_infos.back().points.back();
          break;
        }
        case 1u: {
          point->x = std::stof(linestream);
          break;
        }
        case 2u: {
          point->y = std::stof(linestream);
          break;
        }
        case 3u: {
          point->z = std::stof(linestream);
          break;
        }
        case 4u: {
          info->distance_to_sensor = std::stof(linestream);
          break;
        }
        case 5u: {
          info->ever_free_level_dynamic =
              static_cast<bool>(std::stoi(linestream));
          break;
        }
        case 6u: {
          info->cluster_level_dynamic =
              static_cast<bool>(std::stoi(linestream));
          break;
        }
        case 7u: {
          info->object_level_dynamic = static_cast<bool>(std::stoi(linestream));
          break;
        }
        case 8u: {
          info->ground_truth_dynamic = static_cast<bool>(std::stoi(linestream));
          break;
        }
        case 9u: {
          info->ready_for_evaluation = static_cast<bool>(std::stoi(linestream));
          break;
        }
        case 10u: {
          if (!info->cluster_level_dynamic) {
            // Not part of a cluster.
            break;
          }
          const int cluster_id = std::stoi(linestream);
          Cluster* cluster;
          auto it = cluster_id_to_index.find(cluster_id);
          if (it == cluster_id_to_index.end()) {
            // ID does not yet exist.
            clusters.back().push_back(Cluster());
            cluster = &clusters.back().back();
            cluster->id = cluster_id;
            cluster->valid = true;
            cluster_id_to_index.insert(
                std::pair(cluster_id, clusters.back().size() - 1u));
          } else {
            cluster = &clusters.back().at(it->second);
          }
          // Add the point.
          cluster->points.push_back(clouds.back().size() - 1u);
          break;
        }

        default:
          break;
      }
      row_index++;
    }
  }
  readfile.close();
  return true;
}

}  // namespace dynablox
