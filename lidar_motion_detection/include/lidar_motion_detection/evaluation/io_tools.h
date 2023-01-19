#ifndef LIDAR_MOTION_DETECTION_EVALUATION_IO_TOOLS_H_
#define LIDAR_MOTION_DETECTION_EVALUATION_IO_TOOLS_H_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "lidar_motion_detection/common/types.h"

namespace motion_detection {

/**
 * @brief Save a cloud to a human readable csv file. Repeated calls of this
 * function will append clouds to the same file.
 *
 * @param file_name Full path of output file with extension.
 * @param cloud Point cloud to save.
 * @param cloud_info Associated point infos to save.
 * @param clusters Optionally save the clustering ids.
 * @param cloud_id ID of the cloud to be stored.
 * @returns True if the save operation was successful.
 */
bool saveCloudToCsv(const std::string& file_name, const Cloud& cloud,
                    const CloudInfo& cloud_info,
                    const Clusters& clusters = Clusters(),
                    const int cloud_id = 0);

/**
 * @brief Read all clouds from a given csv file. Each cloud will have a separate
 * entry in the vector.
 *
 * @param file_name Full path of input file with extension.
 * @param clouds Where to store read clouds.
 * @param cloud_infos Where to store read cloud infos.
 * @param clusters Where to store read clusters.
 * @return True if the load operation was successful.
 */
bool loadCloudFromCsv(const std::string& file_name, std::vector<Cloud>& clouds,
                      std::vector<CloudInfo>& cloud_infos,
                      std::vector<Clusters>& clusters);

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_EVALUATION_IO_TOOLS_H_
