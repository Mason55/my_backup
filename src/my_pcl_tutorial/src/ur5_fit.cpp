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
#include <pthread.h>//线程
// #include <tf_conversions/tf/eigen.h>
/*ROS自定义的消息类型*/
#include "point_msgs/Point.h"
#include "Kine.h"
//同步消息接受
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
//读写文件
#include <fstream>

using namespace pcl;
using namespace std;
using namespace Eigen;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
const int BUF_SIZE=4096;

unsigned order (3);//nurbs曲面的阶数
unsigned iterations_fit (20);//拟合的时候迭代的次数

ON_NurbsSurface nurbs (3,false,order,order,order,order);//初始化的nurbs曲面
PointCloudT::Ptr init_cloud (new PointCloudT);
PointCloudT::Ptr curr_cloud (new PointCloudT);

int init_flag = 1;
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
//   Matrix4f init_tool2base_mat = Matrix4f::Identity ();
//   Matrix4f curr_tool2base_mat = Matrix4f::Identity ();
  Matrix4f init2curr_mat = Matrix4f::Identity ();
  Matrix4f curr2init_mat = Matrix4f::Identity ();
  Matrix4f fix_mat = Matrix4f::Identity ();
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
  float tool_in_base_quan[7];//要发布的下一个位置的四元数
  float init_trans[7];//第一个姿态的7个表示位置的xyz,xyzw
  float init_trans_cam2tool[7];//第一个姿态的7个表示位置的xyz,xyzw

  point_msgs::Point visual_output;//可视化输出的点
//   point_msgs::Point move_output;//运动控制输出的点（暂时不考虑）

  float array[600];
  
  ON_3dPoint my_point;
  Kine k_cal;//实例化一个对象，包含四元数和齐次矩阵之间的相互转化函数
//   ON_3dPoint tra_point[4];//路径点
//   ON_3dPoint sor_point;
//   ON_3dPoint pub_point[4];//要发布的路径点（暂时没包含）
  
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

    // fix_mat<<   1  ,   0   ,  0   ,  0.08,//同一个平面内的运动可以跟踪上
    //             0   ,  1 ,   0   ,  0,
    //             0   ,  0  ,   1  ,   0.18,
    //             0   ,  0   ,  0  ,   1;

    // fix_mat<<  -0.9397   , 0.3420     ,    0    ,     0.08,
    //           0.3214  ,  0.8830 ,  -0.3420   ,      0,
    //           -0.1170 ,  -0.3214 ,  -0.9397    ,   0.18,
    //           0   ,  0   ,  0  ,   1;

    fix_mat<<   -1   ,  0   ,   0    ,  0.08,
                0   ,  -1   ,   0    ,  0.07,
                0   ,  0   ,   -1    ,   0.18,
                0   ,  0   ,   0    ,   1;

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
        pcl::fromROSMsg (*cloud_msg, *cloud);
        int cloud_size=cloud->size();

        if(cloud->size()>=cloud_size &&init_flag==1) //确保有足够的数据点，只进入该函数一次
        {
            pthread_mutex_lock(&lock);
            pcl::copyPointCloud(*cloud/*in*/, *init_cloud/*out*/);
            // pthread_mutex_unlock(&lock);
            // // pthread_mutex_lock(&lock);
            // listener.lookupTransform("/base"/*目标坐标系*/, "/tool0_controller"/*source*/,ros::Time(0), init_transform);
            // /*tool 在 base 坐标系下的描述,正运动学*/
            // init_trans[0] = init_transform.getOrigin().x();
            // init_trans[1] = init_transform.getOrigin().y();
            // init_trans[2] = init_transform.getOrigin().z();
            // init_trans[3] = init_transform.getRotation().x();
            // init_trans[4] = init_transform.getRotation().y();
            // init_trans[5] = init_transform.getRotation().z();
            // init_trans[6] = init_transform.getRotation().w();
            // k_cal.quat2Dcm(init_trans, init_tool2base_mat);
            // cout<<"init"<<endl;
            // CloudFit::print4x4Matrix(init_tool2base_mat);

            listener.lookupTransform("/tool0_controller"/*目标坐标系*/, "/camera"/*source*/,ros::Time(0), init_transform_cam2tool);
            /*camera 在 tool 坐标系下的描述*/
            init_trans_cam2tool[0] = init_transform_cam2tool.getOrigin().x();
            init_trans_cam2tool[1] = init_transform_cam2tool.getOrigin().y();
            init_trans_cam2tool[2] = init_transform_cam2tool.getOrigin().z();
            init_trans_cam2tool[3] = init_transform_cam2tool.getRotation().x();
            init_trans_cam2tool[4] = init_transform_cam2tool.getRotation().y();
            init_trans_cam2tool[5] = init_transform_cam2tool.getRotation().z();
            init_trans_cam2tool[6] = init_transform_cam2tool.getRotation().w();
            k_cal.quat2Dcm(init_trans_cam2tool/*7个数表示位置姿态*/, init_cam2tool_mat/*齐次矩阵*/);//调用函数将xyz和四元数转换为齐次矩阵

            pthread_mutex_unlock(&lock);

            pcl::on_nurbs::NurbsDataSurface pca_data;//初始化pca用到的data
            CloudFit::pointCloud2Vector3d (init_cloud, pca_data.interior);//点云数据读入data
            nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &pca_data);//在这里完成了nurbs的初始化
           
            init_flag=0;//初始化完成，设置为0，不再进入该函数
        }
    
    pcl::on_nurbs::NurbsDataSurface data;//这个用于装填后续点云的数据
    pcl::on_nurbs::FittingSurface::Parameter params;//设置曲面拟合所需要的参数，使拟合效果更好
    params.interior_smoothness = 0.2;
    params.interior_weight = 10;
    params.boundary_smoothness = 0.0005;
    params.boundary_weight = 0.01;

pthread_mutex_lock(&lock);
    try/**/
      {   
        listener.lookupTransform("/tool0_controller"/*target_frame*/, "/camera"/*source_frame*/,\
                                                     ros::Time(0), transform);
        // listener.lookupTransform("/base"/*目标坐标系*/, "/tool0_controller"/*source_frame*/,\
        //                                              ros::Time(0), transform2);
        //要定义一个中间数数组变量，把transform转化
        cam2tool_quan[0] = transform.getOrigin().x();
        cam2tool_quan[1] = transform.getOrigin().y();
        cam2tool_quan[2] = transform.getOrigin().z();
        cam2tool_quan[3] = transform.getRotation().x();
        cam2tool_quan[4] = transform.getRotation().y();
        cam2tool_quan[5] = transform.getRotation().z();
        cam2tool_quan[6] = transform.getRotation().w();
        k_cal.quat2Dcm(cam2tool_quan, curr_cam2tool_mat);
        
        
        // tool_in_base_quan[0] = transform2.getOrigin().x();
        // tool_in_base_quan[1] = transform2.getOrigin().y();
        // tool_in_base_quan[2] = transform2.getOrigin().z();
        // tool_in_base_quan[3] = transform2.getRotation().x();
        // tool_in_base_quan[4] = transform2.getRotation().y();
        // tool_in_base_quan[5] = transform2.getRotation().z();
        // tool_in_base_quan[6] = transform2.getRotation().w();//机械臂目标坐标系的四元数
        // k_cal.quat2Dcm(tool_in_base_quan, curr_tool2base_mat);/*正运动学的齐次矩阵*/
        cout<<"init"<<endl;
        CloudFit::print4x4Matrix(init_cam2tool_mat);
        cout<<"current"<<endl;
        CloudFit::print4x4Matrix(curr_cam2tool_mat);

        // tool2cam_mat = cam2tool_mat.inverse();
      }
      catch (tf::TransformException ex)
      {
         ROS_ERROR("%s",ex.what());
      }

    curr2init_mat=fix_mat*(curr_cam2tool_mat.inverse()*init_cam2tool_mat)*fix_mat.inverse();//tool坐标系当前位置到初始位置的变换
    // CloudFit::print4x4Matrix(curr2init_mat);
    init2curr_mat=curr2init_mat.inverse();//tool坐标系初始位置到当前位置的变换
    // CloudFit::print4x4Matrix(init2curr_mat);
pthread_mutex_unlock(&lock);

    //这里要添加一步转化为tool0坐标系下的点云

    pcl::transformPointCloud (*cloud/*in*/, *curr_cloud/*out*/, curr2init_mat);//将相机坐标系下当前的点云转化到相机坐标系下的初始点云位置
    CloudFit::pointCloud2Vector3d (curr_cloud, data.interior);
    pcl::on_nurbs::FittingSurface fit(&data, nurbs);//在初始化的位置进行点云的拟合，nurbs是初始化的平面, fit是拟合后的曲面

    for (unsigned i = 0; i < iterations_fit; ++i)//优化拟合的效果
    {
      fit.assemble (params);
      fit.solve ();
    }

    double u=0.0;
    double v=0.0;
    int NUB_U =6;//设置面上的采样点
    int NUB_V =8;
    int i =0, j=0, k=0;

    for(i=0;i<=NUB_U;i++)//添加等于号是为了包含边界
    {
      for(j=0;j<=NUB_V;j++)
      {
        // my_point = fit.m_nurbs.PointAt(u,v);//u,v等间隔的取点,mypoint
        my_point = nurbs.PointAt(u,v);
        init_paper_point[0] = my_point.x;//都是初始位置相机坐标系下的x , y, z
        init_paper_point[1] = my_point.y;
        init_paper_point[2] = my_point.z;
        init_paper_point[3] = 1.0;
 
 ROS_INFO("init_paper/n");
        cout<<init_paper_point[0]<<endl;
        cout<<init_paper_point[1]<<endl;
        cout<<init_paper_point[2]<<endl;

        curr_paper_point = init2curr_mat*init_paper_point;//转换到相机坐标系下，为了描点
        // curr_paper_point = fix_mat*init2curr_mat*init_paper_point;//转换到相机坐标系下，为了描点
        array[k]   = curr_paper_point[0];
        array[k+1] = curr_paper_point[1];
        array[k+2] = curr_paper_point[2];
ROS_INFO("curr_paper/n");
        cout<<curr_paper_point[0]<<endl;
        cout<<curr_paper_point[1]<<endl;
        cout<<curr_paper_point[2]<<endl;

        k += 3;
        v += (double)1/NUB_V;
      }
      u += (double)1/NUB_U;
      v=0.0;

    }
    std::vector<float> array1(array,array+600);
  
    visual_output.data = array1;

    pub1_.publish (visual_output);/*发布数据点*/
  };
  
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "ur_mesh");
  // ros::NodeHandle nh;
  CloudFit try1;
  
  ros::spin ();
  return 0;
}