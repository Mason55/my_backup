#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/uniform_sampling.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

ros::Publisher pub;


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  PointCloudT::Ptr cloud (new PointCloudT);
  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  pcl::fromROSMsg (*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::PointCloud<int> indices;
  pcl::UniformSampling<PointT> uniform_sampling;
  uniform_sampling.setInputCloud(cloud);
  uniform_sampling.setRadiusSearch(0.02f);
  // uniform_sampling.compute (indices);
  uniform_sampling.filter(*cloud_filtered);

  pcl::PCLPointCloud2 cloud_2_filtered;
  pcl::toPCLPointCloud2 (* cloud_filtered, cloud_2_filtered);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_2_filtered, output);
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "uniform_fliter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("uniform/fliter", 1);

  // Spin
  ros::spin ();
}
