// Copyright 2020 Autonomous Robots Lab, University of Nevada, Reno

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CUDA_PCL_CONVERSIONS_HPP_
#define CUDA_PCL_CONVERSIONS_HPP_

// pcl hdrs
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"

// ROS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// C++
#include <string>

// cuda
#include "cuda_runtime.h"

using namespace std;
using PointT = pcl::PointXYZ;

namespace cuda_pcl_conversions
{
  void cudapclToPcl(float** poutput_d, unsigned int& count, pcl::PointCloud<PointT>::Ptr& point_cloud);
  void pclToCudapcl(pcl::PointCloud<PointT>::Ptr& point_cloud, cudaStream_t* pstream, float** pinput_d, float** poutput_d);
  void cudapclToRos(float** poutput_d, unsigned int& count, sensor_msgs::PointCloud2 &ros_pc2, std::string frame_id);
  void rosToCudapcl(const sensor_msgs::PointCloud2ConstPtr &ros_pc2, cudaStream_t* pstream, float** pinput_d, float** poutput_d);
} // namespace cuda_pcl_conversions

#endif // CUDA_PCL_CONVERSIONS_HPP_
