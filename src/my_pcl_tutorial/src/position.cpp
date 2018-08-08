#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <std_msgs/String.h>
#include <pcl/common/centroid.h>
#include "point_msgs/Point.h"

using namespace pcl;
using namespace std;

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);;

  // Convert to PCL data type
  //pcl::fromROSMsg (*cloud_msg, *cloud);

  // Perform the actual filtering
  Eigen::Vector4f centroid;
  std::stringstream ss;
  point_msgs::Point output;

  //while (ros::ok())
  
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    pcl::fromROSMsg (*cloud_msg, *cloud);
   
    pcl::compute3DCentroid(*cloud,centroid);

    
    ss << "The XYZ coordinates of the centroid are: " <<endl<< "( "\
    <<centroid[0]<<","<<centroid[1]<<","<<centroid[2]<<")"<<std::endl;

    float array[3] = {centroid[0],centroid[1],centroid[2]};
    std::vector<float> array1(array,array+3);
    output.data = array1;

    ROS_INFO("%f", output.data[0]);
    ROS_INFO("%f", output.data[1]);
    ROS_INFO("%f", output.data[2]);


  // Publish the data
  pub.publish (output);
  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "centroid_position");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<point_msgs::Point> ("centroid/position", 1);

  // Spin
  ros::spin ();
}