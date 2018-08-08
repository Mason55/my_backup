#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
  pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered_h (new pcl::PointCloud<PointXYZRGB>);
  pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered_s (new pcl::PointCloud<PointXYZRGB>);
  pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered_i (new pcl::PointCloud<PointXYZRGB>);

  //pcl::fromPCLPointCloud2 (blob, *cloud);
  pcl::fromROSMsg (*cloud_msg, *cloud);

 pcl::ConditionAnd<PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<PointXYZRGB>());


  // build the condition
//----------------------hhhhhhhhhhhhhhhhhhhhhhhh
  int hMax = 0;
  int hMin = -50;
  color_cond->addComparison (pcl::PackedHSIComparison<PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<PointXYZRGB> ("h", pcl::ComparisonOps::LT, hMax)));
   // -128 to 127 corresponds to -pi to pi
  color_cond->addComparison (pcl::PackedHSIComparison<PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<PointXYZRGB> ("h", pcl::ComparisonOps::GT, hMin)));
  //range_red_cond->addComparison (pcl::FieldComparison<PointXYZRGB>::ConstPtr (new pcl::FieldComparison<PointXYZRGB> ("rgb", pcl::ComparisonOps::GT, red1)));
  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> conremo;
  conremo.setCondition (color_cond);
  conremo.setInputCloud (cloud);
  conremo.setKeepOrganized(false);
  // apply filter
  conremo.filter (*cloud_filtered_h);
 
//---------------------------sssssssssssssssssssss
  int sMax = 100;//100以上也有值
  int sMin = 0;
  color_cond->addComparison (pcl::PackedHSIComparison<PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<PointXYZRGB> ("s", pcl::ComparisonOps::LT, sMax)));
  color_cond->addComparison (pcl::PackedHSIComparison<PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<PointXYZRGB> ("s", pcl::ComparisonOps::GT, sMin)));
  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> conremo_s;
  conremo_s.setCondition (color_cond);
  conremo_s.setInputCloud (cloud_filtered_h);
  conremo_s.setKeepOrganized(false);
   // apply filter
  conremo.filter (*cloud_filtered_s);

//-----------------------------iiiiiiiiiiiiiiiiiiiii
  int iMax = 255;
  int iMin = 150;
  color_cond->addComparison (pcl::PackedHSIComparison<PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<PointXYZRGB> ("i", pcl::ComparisonOps::LT, iMax)));
  color_cond->addComparison (pcl::PackedHSIComparison<PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<PointXYZRGB> ("i", pcl::ComparisonOps::GT, iMin)));
  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> conremo_i;
  conremo_i.setCondition (color_cond);
  conremo_i.setInputCloud (cloud_filtered_s);
  conremo_i.setKeepOrganized(false);
  // apply filter
  conremo_i.filter (*cloud_filtered_i);

  pcl::PCLPointCloud2 cloud_2_filtered;
  //pcl::PointCloud<PCLPointCloud2>::Ptr cloud_2_filtered (new pcl::PointCloud<PCLPointCloud2>);
  pcl::toPCLPointCloud2 (* cloud_filtered_i, cloud_2_filtered);
// Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_2_filtered, output);
  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "hsv_pass");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("hsv/pass", 1);

  // Spin
  ros::spin ();
}
