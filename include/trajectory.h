#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__
/**
 * 1. 点云数据融合
 * 2. 点云ROI选取
 * 3. 路径生成
 * 4. 凸包滤波
 */
#include <iostream>
#include <thread>
#include <vector>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

struct BoXPoint {
    double x;
    double y;
};

struct BoxShift {
    double xmin;
    double xmax;
    double ymin;
    double ymax;
};

namespace trajectory_nm {
class Trajectory {
   public:
    Trajectory(ros::NodeHandle nh, tf2_ros::Buffer& tf_);
    ~Trajectory();

   private:
    ros::Subscriber subPath;   // 订阅路径
    ros::Subscriber subCloud;  // 订阅点云
    ros::Publisher pubPath;    // 发布路径
    ros::Publisher pubCloud;
    ros::Publisher pubCmd;
    tf2_ros::Buffer& tf_;  // tf2_ros::Buffer的引用
    // pcl::visualization::PCLVisualizer::Ptr viewer;
    float car_width;
    std::vector<int> polygons;                                 // 凸包索引
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull;          // 凸包
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull_filetered;  // 检测到的障碍点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_path;            // 生成的路径点云
    std::vector<geometry_msgs::Pose> transformed_points;       // base_link坐标系下的导航座标点
    std::vector<BoxShift> box_shift;                           // 存储矩形点云的四个交点

   private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr path_current;    // 路径点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_received;  // 雷达点云
    nav_msgs::Path path_received;                        // 接收到的路径

   public:
    void subPath_callback(const nav_msgs::Path::ConstPtr& msg);             // 路径回调函数
    void subCloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);  // 点云回调函数
    void process();                                                         // 处理函数
    void view_point();                                                      // 可视化
    void init_data();                                                       // 初始化数据
    std::vector<geometry_msgs::Pose> transformPathToBaseLink(const nav_msgs::Path::ConstPtr& msg);//路径点tf转换
    void path_calculation(const std::vector<geometry_msgs::Pose>& points);
    void crophull_filter();
    void box_compute(const geometry_msgs::Pose& center, std::vector<BoXPoint>& box_point);
};
}  // namespace trajectory_nm

#endif