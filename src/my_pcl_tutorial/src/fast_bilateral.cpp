#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/fast_bilateral.h>

using namespace pcl;
using namespace std;

ros::Publisher pub;


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
    pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<PointXYZRGB>);
      pcl::fromROSMsg (*cloud_msg, *cloud);
  // Perform the actual filtering
  pcl::FastBilateralFilter<pcl::PointXYZRGB> fast_filter;
    fast_filter.setInputCloud (cloud);
      fast_filter.setSigmaS (10);
        // fast_filter.setSigmaR (0.005f);
        fast_filter.setSigmaR (0.05f);
          fast_filter.applyFilter (*cloud_filtered);

  pcl::PCLPointCloud2 cloud_2_filtered;
    pcl::toPCLPointCloud2 (* cloud_filtered, cloud_2_filtered);

// Convert to ROS data type
  sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_2_filtered, output);
      pub.publish (output);
}

int
main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "fast_bilateral");
    ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("fast/bilateral", 1);
  // Spin
  ros::spin ();
}
