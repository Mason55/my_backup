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

#include "Kine.h"

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  Kine k;

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloudPtr);
  //pass.setFilterFieldName ("x");
  //pass.setFilterLimits (-0.1, 0.1);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.5, 0.7);
  //pass.setFilterFieldName ("x");
  //pass.setFilterLimits (-1.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (cloud_filtered);

   /*// Perform the actual filtering
  pcl::PCLPointCloud2 cloud_filtered2;
  pcl::PCLPointCloud2Ptr cloudPtr2(cloud_filtered);
  pcl::PassThrough<pcl::PCLPointCloud2> pass2;
  pass2.setInputCloud (cloudPtr2);
  //pass.setFilterFieldName ("x");
  //pass.setFilterLimits (-0.1, 0.1);
  pass2.setFilterFieldName ("y");
  pass2.setFilterLimits (-0.2, 0.2);
  //pass.setFilterFieldName ("x");
  //pass.setFilterLimits (-1.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass2.filter (cloud_filtered2);
*/
  // Convert to ROS data type
  sensor_msgs::PointCloud2 outputOfpass;
  pcl_conversions::fromPCL(cloud_filtered, outputOfpass);

  // Publish the data
  pub.publish (outputOfpass);

  while (ros::ok()){
         k.printHello();
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pass_through");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("pass/through", 1);

  // Spin
  ros::spin ();
}