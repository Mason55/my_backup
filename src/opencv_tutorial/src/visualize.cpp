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

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_1;
  ros::NodeHandle n;
  ros::Subscriber msg_sub;
  image_transport::Publisher image_pub_;

  Vector3f pixel;
  std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> > pixels;


  Vector3f world;
  std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> > worlds;

  Matrix3f K;

  std::vector<float> array;

  const int NUR_POINT = 63;//NUR_POINT=NUB_U*NUB_V

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_1 = it_.subscribe("/kinect2/qhd/image_color", 1, &ImageConverter::imageCb, this);

    msg_sub = n.subscribe("/visual/bsfitting", 1, &ImageConverter::msgCallback,this);

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

    //往vector里面填充数据

    for (int i=0 ;i<NUR_POINT; i++)
    {
      pixels.push_back(pixel);
    }
    for (int i=0;i<NUR_POINT;i++)
    {
      worlds.push_back(world);
    }

    int i=0; int k=0;
    for(i=0;i<NUR_POINT;i++)
    {
      worlds.at(i)[0]=array[k+0];
      worlds.at(i)[1]=array[k+1];
      worlds.at(i)[2]=array[k+2];
      k +=3 ;
      pixels.at(i)=K*worlds.at(i);
      pixels.at(i)=(1/worlds[i][2])/2*pixels.at(i);
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    {
      int NUB_U =7;//
      int NUB_V =9;//
      cv::circle(cv_ptr->image, cv::Point(pixels[0][0], pixels[0][1]), 3, CV_RGB(255,255,255),0);//(0,0)是白点
      cv::circle(cv_ptr->image, cv::Point(pixels[62][0], pixels[62][1]), 3, CV_RGB(0,0,0),0);//(1,1)是黑点
      //显示NUR_POINT个点的位置，-1代表实心,2代表点的半径
      for(int i=0;i<NUR_POINT;i++)
      {
        cv::circle(cv_ptr->image, cv::Point(pixels[i][0], pixels[i][1]), 2, CV_RGB(220,0,0),-1);//显示25个点的位置，-1代表实心,2代表点的半径
      }
    //这5个for循环是连接行
      {
        int k=0;
        for(int i=0;i<NUB_U;i++)
        {
              for(int j=k; j<=k+NUB_U; j++)
              {
                cv::line(cv_ptr->image, cv::Point(pixels[j][0], pixels[j][1]), cv::Point(pixels[j+1][0], pixels[j+1][1]), CV_RGB(255, 255, 0),0.5);
              }
              k +=NUB_V;
        }
      }

    //这5个for循环是连接列
      {
        int k=0;
        for(int i=0;i<NUB_V;i++)
        {
            for(int j=k; j<k+(NUB_U-1)*NUB_V; j+=NUB_V)
            {
              cv::line(cv_ptr->image, cv::Point(pixels[j][0], pixels[j][1]), cv::Point(pixels[j+NUB_V][0], pixels[j+NUB_V][1]), CV_RGB(0, 0, 255),1);
            }
              k +=1;
        }
      }

// //画出目标盒子口的位置
//       cv::rectangle(cv_ptr->image, cvPoint(350, 50), cvPoint(460, 75), cvScalar(0, 0, 255), 3, 4, 0 );
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
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bsfitting_opencv_video");
  ImageConverter ic;
  ros::spin();
  return 0;
}
