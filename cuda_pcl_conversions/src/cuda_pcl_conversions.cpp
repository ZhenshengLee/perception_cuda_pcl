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

#include "cuda_pcl_conversions/cuda_pcl_conversions.h"


namespace cuda_pcl_conversions
{
  void cudapclToPcl(float** poutput_d, unsigned int& count, pcl::PointCloud<PointT>::Ptr& point_cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNew(new pcl::PointCloud<pcl::PointXYZ>);
    cloudNew->width = count;
    cloudNew->height = 1;
    cloudNew->points.resize(cloudNew->width * cloudNew->height);
    for (std::size_t i = 0; i < cloudNew->size(); ++i)
    {
        cloudNew->points[i].x = (*poutput_d)[i * 4 + 0];
        cloudNew->points[i].y = (*poutput_d)[i * 4 + 1];
        cloudNew->points[i].z = (*poutput_d)[i * 4 + 2];
    }
    point_cloud = cloudNew;
  }

  void pclToCudapcl(pcl::PointCloud<PointT>::Ptr& point_cloud, cudaStream_t* pstream, float** pinput_d, float** poutput_d)
  {
    unsigned int nCount = point_cloud->width * point_cloud->height;
    float* inputData = (float*)point_cloud->points.data();

    cudaMallocManaged(pinput_d, sizeof(float) * 4 * nCount, cudaMemAttachHost);
    cudaStreamAttachMemAsync(*pstream, *pinput_d);
    cudaMemcpyAsync(*pinput_d, inputData, sizeof(float) * 4 * nCount, cudaMemcpyHostToDevice, *pstream);
    cudaStreamSynchronize(*pstream);

    cudaMallocManaged(poutput_d, sizeof(float) * 4 * nCount, cudaMemAttachHost);
    cudaStreamAttachMemAsync(*pstream, *poutput_d);
    cudaStreamSynchronize(*pstream);
  }

  void cudapclToRos(float** poutput_d, unsigned int& count, sensor_msgs::PointCloud2 &ros_pc2, std::string frame_id)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cudapclToPcl(poutput_d, count, cloud);
    pcl::toROSMsg(*cloud, ros_pc2);
  }

  void rosToCudapcl(const sensor_msgs::PointCloud2ConstPtr &ros_pc2, cudaStream_t* pstream, float** pinput_d, float** poutput_d)
  {
    pcl::PointCloud<PointT>::Ptr points_cloud{new pcl::PointCloud<PointT>};
    pcl::fromROSMsg(*ros_pc2, *points_cloud);
    pclToCudapcl(points_cloud, pstream, pinput_d, poutput_d);
  }

} // namespace cuda_pcl_conversions
