// prj hdrs
#include "cuda_pcl_conversions/cuda_pcl_conversions.h"

// ros hdrs
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>

using namespace std;

std::string camera_point_topic;
sensor_msgs::PointCloud2 m_pub_cudapcl_pc;

ros::Publisher time_pub;

void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg)
{
    ROS_INFO_STREAM_ONCE("points_callback");

    auto t1 = ros::WallTime::now();
    auto t2 = ros::WallTime::now();

    float* input_d = NULL;
    float* output_d = NULL;
    cudaStream_t stream = NULL;

    t1 = ros::WallTime::now();
    cudaStreamCreate(&stream);
    pcl::PointCloud<PointT>::Ptr points_cloud{new pcl::PointCloud<PointT>};
    pcl::fromROSMsg(*points_msg, *points_cloud);
    cuda_pcl_conversions::pclToCudapcl(points_cloud, &stream, &input_d, &output_d);
    t2 = ros::WallTime::now();

    std_msgs::Int32 time;
    time.data = (t2 - t1).toSec() * 1000.0;
    time_pub.publish(time);
    ROS_INFO_STREAM("rosTocudapcl processing_time: " << (t2 - t1).toSec() * 1000.0 << "[ms]");

    // process only in jetson

    // to ros
    unsigned int count_left = points_cloud->size();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNew(new pcl::PointCloud<pcl::PointXYZ>);
    cuda_pcl_conversions::cudapclToPcl(&output_d, count_left, cloudNew);

    cudaFree(input_d);
    cudaFree(output_d);
    cudaStreamDestroy(stream);

    ROS_INFO_STREAM_ONCE("points_callback end");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cuda_pcl_conversions_test_node");
    ros::NodeHandle private_nh("~");

    private_nh.param("camera3d_point_topic", camera_point_topic, std::string("/points_cloud"));

    auto points_sub = private_nh.subscribe(camera_point_topic, 10, points_callback);
    time_pub = private_nh.advertise<std_msgs::Int32>("/cuda_pcl/ros2cudapcl", 1);

    ros::spin();

    return 0;
}