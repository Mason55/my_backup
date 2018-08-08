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

using namespace pcl;
using namespace std;

ros::Publisher pub;


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
     // pcl::PCLPointCloud2 blob;
  pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
  pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<PointXYZRGB>);

  //pcl::fromPCLPointCloud2 (blob, *cloud);
  pcl::fromROSMsg (*cloud_msg, *cloud);


  // if(cloud->x==NAN || cloud->y==NAN || cloud->z==NAN)
  // {

  // }
  // build the filter


  // pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
  // condrem.setInputCloud (cloud);
  // condrem.setKeepOrganized(true);
  // condrem.filter (*cloud_filtered);

   pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    //conremo.setCondition (color_cond);

    outrem.setRadiusSearch(0.04);//这是我的相机视角 的滤波
    outrem.setMinNeighborsInRadius (160);

    // outrem.setRadiusSearch(0.3);//添加uniform的滤波
    // outrem.setMinNeighborsInRadius (220);

    // apply filter
    outrem.filter (*cloud_filtered);



  pcl::PCLPointCloud2 cloud_2_filtered;
 //pcl::PointCloud<PCLPointCloud2>::Ptr cloud_2_filtered (new pcl::PointCloud<PCLPointCloud2>);

 pcl::toPCLPointCloud2 (* cloud_filtered, cloud_2_filtered);

// Convert to ROS data type

  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_2_filtered, output);

  // Publish the data
  pub.publish (output);
/*
  pcl_msgs::ModelCoefficients ros_coefficients;
     pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub.publish (ros_coefficients)
  */

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "radius_pass");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/radius/pass", 1);

  // Spin
  ros::spin ();
}
