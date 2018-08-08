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
#include "point_msgs/Point.h"

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

  std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> > pixels;
  Vector3f pixel1,pixel2,pixel3,pixel4,pixel5,pixel6,pixel7,pixel8,pixel9,pixel10,pixel11,pixel12,\
    pixel13,pixel14,pixel15,pixel16,pixel17,pixel18,pixel19,pixel20,pixel21,pixel22,pixel23,pixel24,pixel25;

   std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> > worlds;
   Vector3f world1,world2,world3,world4,world5,world6,world7,world8,world9,world10,world11,world12,\
     world13,world14,world15,world16,world17,world18,world19,world20,world21,world22,world23,world24,world25;

  Matrix3f K;

  std::vector<float> array;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_1 = it_.subscribe("/kinect2/qhd/image_color", 1,
      &ImageConverter::imageCb, this);

    msg_sub = n.subscribe("visual/bsfitting", 1, &ImageConverter::msgCallback,this);

    image_pub_ = it_.advertise("bsfitting/opencv/video", 1);

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

  // std::cout<<K<<std::endl;

  // double u=1.0,v=1.0;
  //
  // // pixel.x()=u;
  // pixel[0]=u;
  // pixel[1]=v;
  // pixel[2]=1;
  pixels.push_back(pixel1);  pixels.push_back(pixel2);  pixels.push_back(pixel3);  pixels.push_back(pixel4);
  pixels.push_back(pixel5);  pixels.push_back(pixel6);  pixels.push_back(pixel7);  pixels.push_back(pixel8);
  pixels.push_back(pixel9);  pixels.push_back(pixel10); pixels.push_back(pixel11); pixels.push_back(pixel12);
  pixels.push_back(pixel13);  pixels.push_back(pixel14);  pixels.push_back(pixel15);  pixels.push_back(pixel16);
  pixels.push_back(pixel17);  pixels.push_back(pixel18);  pixels.push_back(pixel19);  pixels.push_back(pixel20);
  pixels.push_back(pixel21);  pixels.push_back(pixel22); pixels.push_back(pixel23); pixels.push_back(pixel24);
  pixels.push_back(pixel25);

  worlds.push_back(world1);  worlds.push_back(world2);  worlds.push_back(world3);  worlds.push_back(world4);
  worlds.push_back(world5);  worlds.push_back(world6);  worlds.push_back(world7);  worlds.push_back(world8);
  worlds.push_back(world9);  worlds.push_back(world10); worlds.push_back(world11); worlds.push_back(world12);
  worlds.push_back(world13);  worlds.push_back(world14);  worlds.push_back(world15);  worlds.push_back(world16);
  worlds.push_back(world17);  worlds.push_back(world18);  worlds.push_back(world19);  worlds.push_back(world20);
  worlds.push_back(world21);  worlds.push_back(world22); worlds.push_back(world23); worlds.push_back(world24);
  worlds.push_back(world25);

  // float array[3] = {msg->data[0],msg->data[1],msg->data[2]};
  // std::vector<float> array1(array,array+3);
  // output.data = array1;

  // std::cout << "msg->data=" << msg->data[0] << ", " << msg->data[1] <<  ", " << msg->data[2] << std::endl;
  // std::cout << "array[0]=" << array[0] << std::endl;
  // std::cout << "array[1]=" << array[1] << std::endl;
  // std::cout << "array[2]=" << array[2] << std::endl;
  //std::cout<<array<<std::endl;
  int i=0; int k=0;
  for(int i=0;i<25;i++)
  {
    worlds.at(i)[0]=array[k+0];
    worlds.at(i)[1]=array[k+1];
    worlds.at(i)[2]=array[k+2];
    k +=3 ;
    pixels.at(i)=K*worlds.at(i);
    pixels.at(i)=(1/worlds[i][2])/2*pixels.at(i);
  }
  // worlds.at(0)[0]=array[0];
  // worlds.at(0)[1]=array[1];
  // worlds.at(0)[2]=array[2];
  // pixels.at(0)=K*worlds.at(0);
  // pixels.at(0)=(1/worlds[0][2])/2*pixels.at(0);
  //
  // worlds.at(1)[0]=array[3];
  // worlds.at(1)[1]=array[4];
  // worlds.at(1)[2]=array[5];
  // pixels.at(1)=K*worlds.at(1);
  // pixels.at(1)=(1/worlds[1][2])/2*pixels.at(1);
  //
  // worlds.at(2)[0]=array[6];
  // worlds.at(2)[1]=array[7];
  // worlds.at(2)[2]=array[8];
  // pixels.at(2)=K*worlds.at(2);
  // pixels.at(2)=(1/worlds[0][2])/2*pixels.at(2);
  //
  // worlds.at(3)[0]=array[9];
  // worlds.at(3)[1]=array[10];
  // worlds.at(3)[2]=array[11];
  // pixels.at(3)=K*worlds.at(3);
  // pixels.at(3)=(1/worlds[0][2])/2*pixels.at(3);
  // pixel[1]=(1/world[2])/2*pixel[1];
  //pixel=pixel/camera_color_factor;

  std::cout<<"0"<<std::endl;
  std::cout<<pixels.at(0)<<std::endl;
  std::cout<<"1"<<std::endl;
  std::cout<<pixels.at(1)<<std::endl;
  std::cout<<"2"<<std::endl;
  std::cout<<pixels.at(2)<<std::endl;
  std::cout<<"3"<<std::endl;
  std::cout<<pixels.at(3)<<std::endl;
  // std::cout<<"4"<<std::endl;
  // std::cout<<pixels.at(12)<<std::endl;
  // std::cout<<"5"<<std::endl;
  // std::cout<<pixels.at(15)<<std::endl;
  // std::cout<<"6"<<std::endl;
  // std::cout<<pixels.at(18)<<std::endl;
  // std::cout<<"7"<<std::endl;
  // std::cout<<pixels.at(21)<<std::endl;


    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    {
      for(int i=0;i<5;i++)
      {
        cv::circle(cv_ptr->image, cv::Point(pixels[i][0], pixels[i][1]), 1, CV_RGB(0,0,255),-1);//显示25个点的位置，-1代表实心,2代表点的半径
      }
//这5个for循环是连接行
      for(int i=0; i<4; i++)
      {
        cv::line(cv_ptr->image, cv::Point(pixels[i][0], pixels[i][1]), cv::Point(pixels[i+1][0], pixels[i+1][1]), CV_RGB(0, 0, 0),2);
      }
      for(int i=5; i<9; i++)
      {
        cv::line(cv_ptr->image, cv::Point(pixels[i][0], pixels[i][1]), cv::Point(pixels[i+1][0], pixels[i+1][1]), CV_RGB(255, 0, 0),2);
      }
      for(int i=10; i<14; i++)
      {
        cv::line(cv_ptr->image, cv::Point(pixels[i][0], pixels[i][1]), cv::Point(pixels[i+1][0], pixels[i+1][1]), CV_RGB(255, 0, 0),1);
      }
      for(int i=15; i<19; i++)
      {
        cv::line(cv_ptr->image, cv::Point(pixels[i][0], pixels[i][1]), cv::Point(pixels[i+1][0], pixels[i+1][1]), CV_RGB(255, 0, 0),1);
      }
      for(int i=20; i<24; i++)
      {
        cv::line(cv_ptr->image, cv::Point(pixels[i][0], pixels[i][1]), cv::Point(pixels[i+1][0], pixels[i+1][1]), CV_RGB(255, 0, 0),1);
      }
//这5个for循环是连接列
      for(int i=0; i<20; i+=5)
      {
        cv::line(cv_ptr->image, cv::Point(pixels[i][0], pixels[i][1]), cv::Point(pixels[i+5][0], pixels[i+5][1]), CV_RGB(0, 0, 255),1);
      }
      for(int i=1; i<21; i+=5)
      {
        cv::line(cv_ptr->image, cv::Point(pixels[i][0], pixels[i][1]), cv::Point(pixels[i+5][0], pixels[i+5][1]), CV_RGB(0, 0, 255),1);
      }
      for(int i=2; i<22; i+=5)
      {
        cv::line(cv_ptr->image, cv::Point(pixels[i][0], pixels[i][1]), cv::Point(pixels[i+5][0], pixels[i+5][1]), CV_RGB(0, 0, 255),1);
      }
      for(int i=3; i<23; i+=5)
      {
        cv::line(cv_ptr->image, cv::Point(pixels[i][0], pixels[i][1]), cv::Point(pixels[i+5][0], pixels[i+5][1]), CV_RGB(0, 0, 255),1);
      }
      for(int i=4; i<24; i+=5)
      {
        cv::line(cv_ptr->image, cv::Point(pixels[i][0], pixels[i][1]), cv::Point(pixels[i+5][0], pixels[i+5][1]), CV_RGB(0, 0, 255),1);
      }
//画出目标盒子口的位置
      cv::rectangle(cv_ptr->image, cvPoint(350, 50), cvPoint(460, 75), cvScalar(0, 0, 255), 3, 4, 0 );
    }


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
  ros::init(argc, argv, "bsfitting_opencv_video");
  ImageConverter ic;
  ros::spin();
  return 0;
}
