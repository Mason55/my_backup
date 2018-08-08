#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>


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
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(60);
  sor.setStddevMulThresh(2);
  sor.filter(*cloud_filtered);

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
  ros::init (argc, argv, "statistical_remove");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("statistical/remove", 1);

  // Spin
  ros::spin ();
}
