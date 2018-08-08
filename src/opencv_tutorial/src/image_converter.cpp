#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/String.h>
// #include <Eigen/Eigen>
#include <Eigen/Dense>
// #include <Eigen/Core>
/*ROS自定义的消息类型*/
#include "point_msgs/msg/Point.msg"

using namespace Eigen;

// 相机内参
const double camera_color_factor = 10;
const double camera_color_cx = 9.3421970832337820e+02;
const double camera_color_cy = 5.3284373211820593e+02;
const double camera_color_fx = 1.0629606164821166e+03;
const double camera_color_fy = 1.0637810683941582e+03;

const double camera_ir_factor = 10;
const double camera_ir_cx = 2.5612408801627015e+02;
const double camera_ir_cy = 2.1124584164854139e+02;
const double camera_ir_fx = 3.6267427007069745e+02;
const double camera_ir_fy = 3.6313664486836473e+02;



static const std::string OPENCV_WINDOW = "Image window";

// void msgCallback(const point_msgs::Point::ConstPtr &msg)
//   {
//     //test_msgs::Test类型里的float32[]数据传到vector
//     std::vector<float> array = msg->data;

//     std::cout << "msg->data[0]=" << msg->data[0] << std::endl;
//     std::cout << "msg->data.size=" << msg->data.size() << std::endl;
//     std::cout << "msg->data=" << msg->data[0] << ", " << msg->data[1] <<  ", " << msg->data[2] << ", " <<  msg->data[3] << ", " <<  msg->data[4] << ", " <<  msg->data[5] << std::endl;

//   }

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_1;
  ros::NodeHandle n;
  ros::Subscriber msg_sub;
  image_transport::Publisher image_pub_;

  Vector3f pixel;
  Vector3f world;
  Matrix3f K;

  std::vector<float> array;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_1 = it_.subscribe("/kinect2/qhd/image_color", 1,
      &ImageConverter::imageCb, this);

    msg_sub = n.subscribe("position", 1, &ImageConverter::msgCallback,this);

    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
// 相机内参
// const double camera_factor = 1000;
// const double camera_cx = 942.3;
// const double camera_cy = 525.7;
// const double camera_fx = 105.2;
// const double camera_fy = 105.2;


    // std::cout << K << std::endl;
  K(0,0)=camera_color_fx;
  K(0,2)=camera_color_cx;
  K(1,1)=camera_color_fy;
  K(1,2)=camera_color_cy;
  K(2,2)=1;

  std::cout<<K<<std::endl;

  double u=0.0,v=0.0;

  // pixel.x()=u;
  pixel[0]=u;
  pixel[1]=v;
  pixel[2]=1;


  // float array[3] = {msg->data[0],msg->data[1],msg->data[2]};
  // std::vector<float> array1(array,array+3);
  // output.data = array1;

  // std::cout << "msg->data=" << msg->data[0] << ", " << msg->data[1] <<  ", " << msg->data[2] << std::endl;
  // std::cout << "array[0]=" << array[0] << std::endl;
  // std::cout << "array[1]=" << array[1] << std::endl;
  // std::cout << "array[2]=" << array[2] << std::endl;
  //std::cout<<array<<std::endl;
  world[0]=array[0];
  world[1]=array[1];
  world[2]=array[2];


  pixel=K*world;
  pixel=pixel*(1/world[2])/2;
  //pixel=pixel/camera_color_factor;

  std::cout<<std::endl;
  std::cout<<pixel<<std::endl;


    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(pixel[0], pixel[1]), 10, CV_RGB(0,0,255));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void msgCallback(const point_msgs::Point::ConstPtr &msg)
  {
    //test_msgs::Test类型里的float32[]数据传到vector
    array = msg->data;

     // std::cout << "array[0]=" << array[0] << std::endl;
    // std::cout << "msg->data.size=" << msg->data.size() << std::endl;
     // std::cout << "msg->data=" << msg->data[0] << ", " << msg->data[1] <<  ", " << msg->data[2] << std::endl;

  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
