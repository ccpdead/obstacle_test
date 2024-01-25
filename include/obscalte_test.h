#ifndef __TRAJECTORY_TEST_H__
#define __TRAJECTORY_TEST_H__
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

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

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
    geometry_msgs::Pose center;
};

namespace trajectory_nm {
class Trajectory {
   public:
    Trajectory(ros::NodeHandle nh, tf2_ros::Buffer& tf_);
    ~Trajectory();

   private:
    ros::Subscriber subPath;                                   // 订阅路径
    ros::Subscriber subCloud;                                  // 订阅点云
    ros::Publisher pubPath;                                    // 发布路径
    ros::Publisher pubCloud;                                   // 滤波后的点云
    ros::Publisher pubCmd;                                     // 发布速度指令
    ros::Publisher filted_point;                               // 发布滤波的四个点云点
    tf2_ros::Buffer& tf_;                                      // tf2_ros::Buffer的引用
    ros::Publisher car_marker_pub;                             // 车体轮廓marker
    ros::Publisher arm_marker_pub;                             // 机械臂轮廓marker
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
    void subPath_callback(const nav_msgs::Path::ConstPtr& msg);                                     // 路径回调函数
    void subCloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);                          // 点云回调函数
    void process();                                                                                 // 处理函数
    void view_point();                                                                              // pcl点云可视化
    void init_data();                                                                               // 初始化数据
    std::vector<geometry_msgs::Pose> transformPathToBaseLink(const nav_msgs::Path::ConstPtr& msg);  // 路径点tf转换
    void path_calculation(const std::vector<geometry_msgs::Pose>& points);                          // 路径计算与生成
    void crophull_filter(std::vector<pcl::PointIndices>& point_index);                              // 滤波与聚类
    void box_compute(const geometry_msgs::Pose& center, std::vector<BoXPoint>& box_point);          // 根据路径计算车体立方体
    void marker_pub(float car_min_x,
                    float car_max_x,
                    float car_min_y,
                    float car_max_y,
                    float arm_min_x,
                    float arm_max_x,
                    float arm_min_y,
                    float arm_max_y,
                    geometry_msgs::Pose center);
};
}  // namespace trajectory_nm

#endif