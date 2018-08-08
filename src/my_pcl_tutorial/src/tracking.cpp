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

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  /*/////////////////////////////////////
    voxel_grid
  *//////////////////////////////////////

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_voxel_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_voxel_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 voxel_output;
  pcl_conversions::fromPCL(cloud_voxel_filtered, voxel_output);

  // Publish the data
  pub.publish (voxel_output);

  /*/////////////////////////////////////
    passthrough
  *//////////////////////////////////////

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr2(cloud2);
  pcl::PCLPointCloud2 cloud_pass_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(voxel_output, *cloud2);

  // Perform the actual filtering
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloudPtr2);
  //pass.setFilterFieldName ("x");
  //pass.setFilterLimits (0, 0.001);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.2, 0.0);
  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits (-1.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (cloud_pass_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 pass_output;
  pcl_conversions::fromPCL(cloud_pass_filtered, pass_output);

  // Publish the data
  pub.publish (pass_output);
 
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("tracking", 1);

  // Spin
  ros::spin ();
}