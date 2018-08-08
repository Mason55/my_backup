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

void HSVtoRGB(int *r, int *g, int  *b, int h, int s, int v)  
    {  
        // convert from HSV/HSB to RGB color  
        // R,G,B from 0-255, H from 0-360, S,V from 0-100  
        // ref http://colorizer.org/  
      
        // The hue (H) of a color refers to which pure color it resembles  
        // The saturation (S) of a color describes how white the color is  
        // The value (V) of a color, also called its lightness, describes how dark the color is  
      
        int i;  
      
      
        float RGB_min, RGB_max;  
        RGB_max = v*2.55f;  
        RGB_min = RGB_max*(100 - s)/ 100.0f;  
      
        i = h / 60;  
        int difs = h % 60; // factorial part of h  
      
        // RGB adjustment amount by hue   
        float RGB_Adj = (RGB_max - RGB_min)*difs / 60.0f;  
      
        switch (i) {  
        case 0:  
            *r = RGB_max;  
            *g = RGB_min + RGB_Adj;  
            *b = RGB_min;  
            break;  
        case 1:  
            *r = RGB_max - RGB_Adj;  
            *g = RGB_max;  
            *b = RGB_min;  
            break;  
        case 2:  
            *r = RGB_min;  
            *g = RGB_max;  
            *b = RGB_min + RGB_Adj;  
            break;  
        case 3:  
            *r = RGB_min;  
            *g = RGB_max - RGB_Adj;  
            *b = RGB_max;  
            break;  
        case 4:  
            *r = RGB_min + RGB_Adj;  
            *g = RGB_min;  
            *b = RGB_max;  
            break;  
        default:        // case 5:  
            *r = RGB_max;  
            *g = RGB_min;  
            *b = RGB_max - RGB_Adj;  
            break;  
        }  
    }


    //to find the max one of the r1/r2 g1/g2 b1/b2  2is bigger than 1
    void max(int & x1,  int & x2)
    {
      if(x1>x2)
      {
        x1=x1+x2;
        x2=x1-x2;
        x1=x1-x2;
      }
    }





void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
      
     // pcl::PCLPointCloud2 blob;
  pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
  pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<PointXYZRGB>);

  //pcl::fromPCLPointCloud2 (blob, *cloud);
  pcl::fromROSMsg (*cloud_msg, *cloud);


                       /*pcl::ConditionAnd<PointXYZRGB>::Ptr range_cond (new
                          pcl::ConditionAnd<PointXYZRGB> ());
                            uint8_t r1 = 0, g1 = 0, b1 = 0;    // Example: black color
                            uint32_t rgb1 = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);
                            float p1= *reinterpret_cast<float*>(&rgb1);

                            uint8_t r2 = 255, g2 = 255, b2 = 255;    // Example: white color
                            uint32_t rgb2 = ((uint32_t)r2 << 16 | (uint32_t)g2 << 8 | (uint32_t)b2);
                            float p2= *reinterpret_cast<float*>(&rgb2);

                        range_cond->addComparison (pcl::FieldComparison<PointXYZRGB>::ConstPtr (new
                          pcl::FieldComparison<PointXYZRGB> ("rgb", pcl::ComparisonOps::GT, p1)));
                        range_cond->addComparison (pcl::FieldComparison<PointXYZRGB>::ConstPtr (new
                          pcl::FieldComparison<PointXYZRGB> ("rgb", pcl::ComparisonOps::LT, p2)));
                          */
  
  // build the condition
  int rMax = 200;
  int rMin = 160;
  int gMax = 200;
  int gMin = 160;
  int bMax = 200;
  int bMin = 160;

  HSVtoRGB(&rMin, &gMin, &bMin, 0, 0, 0);

  HSVtoRGB(&rMax, &gMax, &bMax, 200, 50, 50);

  max(rMin,rMax);
  max(gMin,gMax);
  max(bMin,bMax);



  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin)));

  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
  condrem.setInputCloud (cloud);
  condrem.setKeepOrganized(true);

  // apply filter
  condrem.filter (*cloud_filtered);

  
   
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
  ros::init (argc, argv, "conditional_pass");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("conditional/pass", 1);

  // Spin
  ros::spin ();
}