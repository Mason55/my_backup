#include <ros/ros.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <vector>
#include <Eigen/Dense>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/from_meshes.h>
#include <string.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <pthread.h>//线程 用于保护数据的复制
// #include <opennurbs_nurbssurface.h>
// #include <tf_conversions/tf/eigen.h>
/*ROS自定义的消息类型*/
#include "point_msgs/Point.h"
#include "Kine.h"
//同步消息接受
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
//读写文件
#include <fstream>
#include <iomanip>

using namespace pcl;
using namespace std;
using namespace Eigen;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
const int BUF_SIZE=4096;

unsigned order (3);//nurbs曲面的阶数
unsigned refinement (2);
unsigned iterations_fit (5);//拟合的时候迭代的次数

ON_NurbsSurface nurbs (3,false,order,order,order,order);//初始化的nurbs曲面
PointCloudT::Ptr init_cam_cloud (new PointCloudT);
PointCloudT::Ptr init_tool_cloud (new PointCloudT);
PointCloudT::Ptr curr2init_cloud (new PointCloudT);

int init_flag = 1;
int flag_2 = 0;
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

class CloudFit
{
  ros::NodeHandle nh_;
  ros::Subscriber sub1_;//曲面拟合
  ros::Publisher pub1_;//发布绘图点
  ros::Publisher pub2_;//发布四元数
  ros::Publisher pub3_;//测试发布点云

  /*init 代表初始化的量;curr代表当前状态量； */
  Matrix4f cam2base_mat = Matrix4f::Identity (); 
  Matrix4f parper2tool_mat = Matrix4f::Identity ();
  Matrix4f parper2base_mat = Matrix4f::Identity ();
  Matrix4f target2base_mat = Matrix4f::Identity ();
  Matrix4f parper2tool_mat_2 = Matrix4f::Identity ();
  Matrix4f init_tool2base_mat = Matrix4f::Identity ();
  Matrix4f curr_tool2base_mat = Matrix4f::Identity ();
  Matrix4f init2curr_mat = Matrix4f::Identity ();
  Matrix4f curr2init_mat = Matrix4f::Identity ();
  Matrix4f curr2init_2_mat = Matrix4f::Identity ();
  Matrix4f par2tool_mat = Matrix4f::Identity ();
  Matrix4f init_cam2tool_mat = Matrix4f::Identity ();
  Matrix4f curr_cam2tool_mat = Matrix4f::Identity ();
  Vector4f init_paper_point;
  Vector4f curr_paper_point;

  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::StampedTransform transform2;
  tf::StampedTransform init_transform;
  tf::StampedTransform init_transform_cam2tool;

  float cam2tool_quan[7];
  float tool_in_base_quan[7];//
  float curr_trans[8];//要发布的姿态和运行时间
  float init_trans[7];//第一个姿态的7个表示位置的xyz,xyzw
  float init_trans_cam2tool[7];//第一个姿态的7个表示位置的xyz,xyzw

  point_msgs::Point visual_output;//可视化输出的点
  point_msgs::Point move_output_1;//运动控制输出的点（暂时不考虑）
  point_msgs::Point move_output_2;

  float array[600];

 public: 
  ON_3dPoint my_point;
  Kine k_cal;//实例化一个对象，包含四元数和齐次矩阵之间的相互转化函数
  ON_3dPoint box_point_1;//路径点
  ON_3dPoint point_0;//指定1和2来到达指定位置 1到达指定位置的上方
  ON_3dPoint point_1;//插入纸片
  ON_3dPoint point_2;
  ON_3dPoint point_3;//指定1和2来到达指定位置 1到达指定位置的上方
  ON_3dPoint point_4;//插入纸片
  ON_3dPoint point_5;
  ON_3dPoint edge_point;//3是纸边缘的点
  ON_3dPoint edge_in_base_point;//4是基座坐标系下的点
  ON_3dPoint aim_point_1;//5是目标最终点
  ON_3dPoint aim_in_base_point_1;//6是目标最终点
  Vector4f edge_in_point;
  Vector4f edge_out_point;
  Vector4f aim_in_point_1;
  Vector4f aim_out_point_1;
  ON_3dPoint aim_point_2;//5是目标最终点
  ON_3dPoint aim_in_base_point_2;//6是目标最终点
  Vector4f aim_in_point_2;
  Vector4f aim_out_point_2;
  double move_quan[7];//要发布的下一个位置的四元数

public:
  CloudFit()
  {
    /*订阅/voxel/grid，更新拟合的nurbs*/
    sub1_ = nh_.subscribe ("/voxel/grid", 1, &CloudFit::surFit, this);
    /*发布点用于绘制曲面*/
    pub1_ = nh_.advertise<point_msgs::Point> ("/visual/bsfitting", 1);
    /*发布下一阶段的位移*/
    pub2_ = nh_.advertise<point_msgs::Point> ("/ur/move/tra", 1);//发布的是每一段的位移
    /*用于测试初始化点云，不影响后续结果*/
    /*pub3_ = nh_.advertise<sensor_msgs::PointCloud2> ("dianyun", 1);*/
    cam2base_mat<<-0.0213  , -0.8264  ,  -0.5626  ,  1.1210,//这是把手眼标定的矩阵拿过来用了
                  -0.9997  ,  0.0264  ,  -0.0009  , -0.1194,
                   0.0156  ,  0.5624  ,  -0.8267  ,  1.0983,
                  -0.0000  , -0.0000  ,   0.0000  ,  1.0000;

    par2tool_mat<< 0   ,       1  ,   0  ,        0,//这个 的效果是调整出来的,还不错 rotx(32)
                    0.8480   ,   0  ,  -0.5878  ,     0,
                    0.5299  ,    0  ,   0.8480  ,    0,
                    0   ,    0  ,   0  ,       1;

    // par2tool_mat<< 0   ,       1  ,   0  ,        0,//这个 的效果是调整出来的,还不错 rotx(32)
    //             0.8480   ,   0  ,  -0.5878  ,     0,
    //             0.5299  ,    0  ,   0.8480  ,    0,
    //             0   ,    0  ,   0  ,       1;

    init_paper_point<<0.0,0.0,0.0,1.0;
    curr_paper_point<<0.0,0.0,0.0,1.0;

    ROS_INFO("CloudFit works/n");
  };
  ~CloudFit(){    };
  
  void pointCloud2Vector3d (pcl::PointCloud<PointT>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
  {/*点云数据填充到data中*/
    for (unsigned i = 0; i < cloud->size (); i++)  
    {
      PointT &p = cloud->at (i);
      if (!pcl_isnan (p.x) && !pcl_isnan (p.y) && !pcl_isnan (p.z))
          data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
    }
  };

  void print4x4Matrix (const Eigen::Matrix4f & matrix)
  {
    printf ("    | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2), matrix (0, 3));
    printf ("R = | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2), matrix (1, 3));
    printf ("    | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2), matrix (2, 3));
    printf ("    | %6.3f %6.3f %6.3f %6.3f  | \n", matrix (3, 0), matrix (3, 1), matrix (3, 2), matrix (3, 3));
  }

  void surFit (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)/*该函数被调用后不断的循环*/
  {
        PointCloudT::Ptr cloud (new PointCloudT);
        PointCloudT::Ptr curr_tool_cloud (new PointCloudT);
        pcl::fromROSMsg (*cloud_msg, *cloud);
        int cloud_size=cloud->size();

        if(cloud->size()>=cloud_size &&init_flag==1) //确保有足够的数据点，只进入该函数一次
        {
pthread_mutex_lock(&lock);
            pcl::copyPointCloud(*cloud/*in*/, *init_cam_cloud/*out*/);
            // pthread_mutex_unlock(&lock);
            // // pthread_mutex_lock(&lock);
            listener.lookupTransform("/base"/*目标坐标系*/, "/tool0_controller"/*source*/,ros::Time(0), init_transform);
            /*tool 在 base 坐标系下的描述,正运动学*/
            init_trans[0] = init_transform.getOrigin().x();
            init_trans[1] = init_transform.getOrigin().y();
            init_trans[2] = init_transform.getOrigin().z();
            init_trans[3] = init_transform.getRotation().x();
            init_trans[4] = init_transform.getRotation().y();
            init_trans[5] = init_transform.getRotation().z();
            init_trans[6] = init_transform.getRotation().w();
            k_cal.quat2Dcm(init_trans, init_tool2base_mat);
            // cout<<"init_tool2base_mat:"<<endl;
            // CloudFit::print4x4Matrix(init_tool2base_mat);
pthread_mutex_unlock(&lock);

            pcl::on_nurbs::NurbsDataSurface pca_data;//初始化pca用到的data
            CloudFit::pointCloud2Vector3d (init_cam_cloud, pca_data.interior);//点云数据读入data
            nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &pca_data);//在这里完成了nurbs在tool坐标系下的初始化
           
            init_flag=0;//初始化完成，设置为0，不再进入该函数
        }
    
    pcl::on_nurbs::NurbsDataSurface data;//这个用于装填后续点云的数据
    pcl::on_nurbs::FittingSurface::Parameter params;//设置曲面拟合所需要的参数，使拟合效果更好
    params.interior_smoothness = 2;
    params.interior_weight = 1;
    params.boundary_smoothness = 0.006;
    params.boundary_weight = 0.5;
    // params.interior_regularisation = 0;
    // params.boundary_regularisation = 0;

    // params.interior_smoothness = 2;
    // params.interior_weight = 2;
    // params.boundary_smoothness = 0;
    // params.boundary_weight = 10;

pthread_mutex_lock(&lock);
    try/**/
      {   
        listener.lookupTransform("/base"/*目标坐标系*/, "/tool0_controller"/*source_frame*/,\
                                                     ros::Time(0), transform2);
        tool_in_base_quan[0] = transform2.getOrigin().x();
        tool_in_base_quan[1] = transform2.getOrigin().y();
        tool_in_base_quan[2] = transform2.getOrigin().z();
        tool_in_base_quan[3] = transform2.getRotation().x();
        tool_in_base_quan[4] = transform2.getRotation().y();
        tool_in_base_quan[5] = transform2.getRotation().z();
        tool_in_base_quan[6] = transform2.getRotation().w();//机械臂目标坐标系的四元数
        k_cal.quat2Dcm(tool_in_base_quan, curr_tool2base_mat);/*正运动学的齐次矩阵*/
        // cout<<"curr_tool2base_mat:"<<endl;
        // CloudFit::print4x4Matrix(curr_tool2base_mat);
      }
      catch (tf::TransformException ex)
      {
         ROS_ERROR("%s",ex.what());
      }
pthread_mutex_unlock(&lock);
    
    curr2init_mat=par2tool_mat*curr_tool2base_mat*init_tool2base_mat.inverse()*par2tool_mat.inverse();//buxing 
    init2curr_mat=curr2init_mat.inverse();//tool坐标系初始位置到当前位置的变换
    pcl::transformPointCloud (*cloud/*in*/, *curr2init_cloud/*out*/, curr2init_mat);//将相机坐标系下当前的点云转化到相机坐标系下的初始点云位置
    CloudFit::pointCloud2Vector3d (curr2init_cloud, data.interior);
    pcl::on_nurbs::FittingSurface fit(&data, nurbs);//在初始化的位置进行点云的拟合，nurbs是初始化的平面, fit是拟合后的曲面

    for (unsigned i = 0; i < iterations_fit; ++i)//优化拟合的效果
    {
      fit.assemble (params);
      fit.solve (0.46);
    }
    //再添加一下优化拟合的

//************************计算点
// /*\测试目标点   
   
    /*在这里设定最终的期望点的位置*/
    aim_point_1 = fit.m_nurbs.PointAt(0,0.5);
    aim_in_point_1[0] = aim_point_1.x;
    aim_in_point_1[1] = aim_point_1.y;
    aim_in_point_1[2] = aim_point_1.z;
    aim_in_point_1[3] = 1;
    aim_out_point_1 = cam2base_mat*init2curr_mat*aim_in_point_1;
    aim_in_base_point_1.x = aim_out_point_1[0];
    aim_in_base_point_1.y = aim_out_point_1[1];
    aim_in_base_point_1.z = aim_out_point_1[2];
    cout<<"aim_in_base_point_1 x:"<<aim_in_base_point_1.x<<endl;
    cout<<"aim_in_base_point_1 y:"<<aim_in_base_point_1.y<<endl;
    cout<<"aim_in_base_point_1 z:"<<aim_in_base_point_1.z<<endl;

    aim_point_2 = fit.m_nurbs.PointAt(1,0.5);
    aim_in_point_2[0] = aim_point_2.x;
    aim_in_point_2[1] = aim_point_2.y;
    aim_in_point_2[2] = aim_point_2.z;
    aim_in_point_2[3] = 1;
    aim_out_point_2 = cam2base_mat*init2curr_mat*aim_in_point_2;
    aim_in_base_point_2.x = aim_out_point_2[0];
    aim_in_base_point_2.y = aim_out_point_2[1];
    aim_in_base_point_2.z = aim_out_point_2[2];
    cout<<"aim_in_base_point_2 x:"<<aim_in_base_point_2.x<<endl;
    cout<<"aim_in_base_point_2 y:"<<aim_in_base_point_2.y<<endl;
    cout<<"aim_in_base_point_2 z:"<<aim_in_base_point_2.z<<endl;

    double secs = ros::Time::now().toSec();
    static double init_secs = ros::Time::now().toSec();
    double timer =secs - init_secs;

    ofstream outFile;
    /*ios::trunc表示在打开文件前将文件清空,由于是写入,文件不存在则创建*/
    outFile.open("/home/mason/catkin_ws/data/TraOfRobotAndAim/aim_point_1_20180826.txt",ios::app);
    // ros::NodeHandle nh;
    outFile<<timer<<" "<<aim_in_base_point_1.x<<" "<<aim_in_base_point_1.y<<" "<<aim_in_base_point_1.z<<endl;
    outFile.close();//关闭文件

    ofstream outFile;
    /*ios::trunc表示在打开文件前将文件清空,由于是写入,文件不存在则创建*/
    outFile.open("/home/mason/catkin_ws/data/TraOfRobotAndAim/aim_point_2_20180826.txt",ios::app);
    // ros::NodeHandle nh;
    outFile<<timer<<" "<<aim_in_base_point_2.x<<" "<<aim_in_base_point_2.y<<" "<<aim_in_base_point_2.z<<endl;
    outFile.close();//关闭文件
    // cout<<"---------"<<endl;

    /*box是最终要到达的位置*/
    box_point_1.x=0.5300; box_point_1.y=-0.2800; box_point_1.z=-0.032;

    double aim_x = box_point_1.x - aim_in_base_point_1.x;
    double aim_y = box_point_1.y - aim_in_base_point_1.y;
    double aim_z = box_point_1.z - aim_in_base_point_1.z;

    ROS_INFO("display:");
    // cout<<"edge_z"<<edge_z<<endl;
    // cout<<"aim_x:"<<aim_x<<endl;
    // cout<<"aim_y:"<<aim_y<<endl;
    // cout<<"aim_z:"<<aim_z<<endl;
    // cout<<"---------"<<endl;
   
    /*位置可以变,姿态不能变*/
    curr_trans[0]=tool_in_base_quan[0];
    curr_trans[1]=tool_in_base_quan[1];
    curr_trans[2]=tool_in_base_quan[2];
    curr_trans[3]=init_trans[3];
    curr_trans[4]=init_trans[4];
    curr_trans[5]=init_trans[5];
    curr_trans[6]=init_trans[6];
    curr_trans[7]=0.0;

    /*初始化两种情况,从中选出较低者作为目标*/
    point_0.x=0.4570; point_0.y=0.0675; point_0.z=0.23;//提起来
    point_1.x=0.5358; point_1.y=-0.1030; point_1.z=0.2420;//对准了
    point_2.x=0.5280; point_2.y=-0.3000; point_2.z=0.3240;//移动过去
    point_3.x=0.5280; point_3.y=-0.2900; point_3.z=0.2500;//提起来
    point_4.x=0.5280; point_4.y=-0.2590; point_4.z=0.2250;//放下去 一推

    double p0_distance = abs(tool_in_base_quan[2]-point_0.z);
    double p1_distance = abs(tool_in_base_quan[0]-point_1.x);
    double p2_distance = abs(tool_in_base_quan[1]-point_2.y);
    double p3_distance = abs(tool_in_base_quan[2]-point_3.z);
    double p4_distance = abs(tool_in_base_quan[2]-point_3.z);

    if(flag_2==0)
    {
        curr_trans[2]=point_0.z;
        curr_trans[7]=5;
        std::vector<float> p1_array(curr_trans,curr_trans+8);
        move_output_1.data = p1_array;
        if (p0_distance<0.002)flag_2=1;
    }
      if(flag_2==1)
    {
        curr_trans[0]=point_1.x;
        curr_trans[7]=5;
        std::vector<float> p2_array(curr_trans,curr_trans+8);
        move_output_1.data = p2_array;
        if (p1_distance<0.003)flag_2=2;
    }
      if(flag_2==2)
    {
        curr_trans[1]=point_2.y;
        curr_trans[7]=12;
        std::vector<float> p2_array(curr_trans,curr_trans+8);
        move_output_1.data = p2_array;
        if (p2_distance<0.003)flag_2=3;
    }
     if(flag_2==3)
    {
        curr_trans[2]=point_3.z;
        curr_trans[7]=2;
        std::vector<float> p2_array(curr_trans,curr_trans+8);
        move_output_1.data = p2_array;
        if (p3_distance<0.003)flag_2=4;
    }
     if(flag_2==4)
    {
        curr_trans[1]=point_4.y;
        curr_trans[2]=point_4.z;
        curr_trans[7]=2;
        std::vector<float> p2_array(curr_trans,curr_trans+8);
        move_output_1.data = p2_array;
        if (p3_distance<0.003)flag_2=4;
    }
      cout<<"flag_2: "<<flag_2<<endl;
      pub2_.publish (move_output_1);

    ofstream outFile_2;
    /*ios::trunc表示在打开文件前将文件清空,由于是写入,文件不存在则创建*/
    outFile_2.open("/home/mason/catkin_ws/data/TraOfRobotAndAim/arm_end_point_10180820.txt",ios::app);
    // ros::NodeHandle nh;
    outFile_2<<timer<<" "<<curr_trans[0]<<" "<<curr_trans[1]<<" "<<curr_trans[2]<<endl;
    outFile_2.close();//关闭文件
    cout<<"---------"<<endl;

//******这些部分是用来采点拟合的
     double u=0.0;
     double v=0.0;
     int NUB_U =10;//设置面上的采样点
     int NUB_V =7;
     int i =0, j=0, k=0;

     for(i=0;i<=NUB_U;i++)//添加等于号是为了包含边界
     {
        for(j=0;j<=NUB_V;j++)
        {
                my_point = fit.m_nurbs.PointAt(u,v);//u,v等间隔的取点,mypoint
                // my_point = nurbs.PointAt(u,v);
                init_paper_point[0] = my_point.x;//都是初始位置相机坐标系下的x , y, z
                init_paper_point[1] = my_point.y;
                init_paper_point[2] = my_point.z;
                init_paper_point[3] = 1.0;
                curr_paper_point = init2curr_mat*init_paper_point;//转换到相机坐标系下，为了描点
                array[k]   = curr_paper_point[0];
                array[k+1] = curr_paper_point[1];
                array[k+2] = curr_paper_point[2];
                k += 3;
                v += (double)1/NUB_V;
        }
        u += (double)1/NUB_U;
        v=0.0;
     }

    std::vector<float> array_visual(array,array+600);
    visual_output.data = array_visual;
    pub1_.publish (visual_output);/*发布数据点*/
  };
  
};

int main (int argc, char** argv)
{
    ros::init (argc, argv, "ur_mesh");
    CloudFit try1;
    ros::spin ();
    return 0;
}

