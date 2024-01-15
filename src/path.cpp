#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <vector>

class RobotController {
   public:
    RobotController() {
        /*-----------------------------初始化ROS节点------------------------------*/
        ros::NodeHandle nh;
        odom_subscriber = nh.subscribe("/odom", 1, &RobotController::odomCallback, this);
        trajectory_path_pub = nh.advertise<sensor_msgs::PointCloud2>("trajectory_path_pub", 10);

        rslidar_subscriber = nh.subscribe("/lidar_fusion", 1, &RobotController::rslidarCallback, this);
        trajectory_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("trajectory_cloud_pub", 10);

        /*------------------------初始化数据类型--------------------------*/
        trajectory_point.reset(new pcl::PointCloud<pcl::PointXYZ>());  // 初始化变量
        cloud_rslidar.reset(new pcl::PointCloud<pcl::PointXYZ>());     // 初始化变量
        cloud_trajectory.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 提取线速度和角速度
        double linear_velocity = msg->twist.twist.linear.x;
        double angular_velocity = msg->twist.twist.angular.z;
        if (abs(linear_velocity) >= 0.002)
            // 生成机器人轨迹曲线
            generateTrajectory(linear_velocity, angular_velocity);
    }

    void rslidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::fromROSMsg(*msg, *cloud_rslidar);  // 将ROS消息转换为PCL格式
    }

    /**
     * 路径生成
     */
    void generateTrajectory(double linear_velocity, double angular_velocity) {
        // 计算转弯半径
        double turning_radius = linear_velocity / angular_velocity;
        printf("turning_radius:%f\n", turning_radius);
        // 生成机器人轨迹曲线
        publishTrajectory_point(turning_radius, 0.8f);  // 转弯半径，车身宽度
    }

    /**
     * 发布路径
     */
    void publishTrajectory_point(float r, float width) {
        // 直线行驶
        if (abs(r) > 50.0) {
            std::vector<geometry_msgs::Point> points;
            trajectory_point->push_back(pcl::PointXYZ(3.0, width, 0.0));
            trajectory_point->push_back(pcl::PointXYZ(0.0, width, 0.0));
            trajectory_point->push_back(pcl::PointXYZ(3.0, -width, 0.0));
            trajectory_point->push_back(pcl::PointXYZ(0.0, -width, 0.0));
        } else {  // 曲线行驶
            float R = 0.05 / r;
            printf("R:%f\n", R);
            for (double theta = M_PI; theta <= M_PI * 2; theta += abs(R)) {
                geometry_msgs::Point point;
                // 内圆
                point.x = cos(theta) * (r - width);
                point.y = sin(theta) * (r - width) + r;
                point.z = 0.0;
                if (point.x <= 3.0 && point.x >= 0)
                    trajectory_point->push_back(pcl::PointXYZ(point.x, point.y, point.z));
                // 外圆
                point.x = cos(theta) * (r + width);
                point.y = sin(theta) * (r + width) + r;
                if (point.x <= 3.0 && point.x >= 0)
                    trajectory_point->push_back(pcl::PointXYZ(point.x, point.y, point.z));
            }
        }

        // 检测障碍物点云
        cloud_trajectory = convexHullFilter(trajectory_point, cloud_rslidar);
        sensor_msgs::PointCloud2 cloud_trajectory_ros;  // 障碍点云
        pcl::toROSMsg(*cloud_trajectory, cloud_trajectory_ros);
        cloud_trajectory_ros.header.frame_id = "mid360_frame";
        cloud_trajectory_ros.header.stamp = ros::Time::now();

        sensor_msgs::PointCloud2 trajectory_path;  // 路径点云
        pcl::toROSMsg(*trajectory_point, trajectory_path);
        trajectory_path.header.frame_id = "mid360_frame";
        trajectory_path.header.stamp = ros::Time::now();

        trajectory_path_pub.publish(trajectory_path);
        trajectory_cloud_pub.publish(cloud_trajectory_ros);
        trajectory_point->clear();
    }

    // 路径点云滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHullFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr path_input,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered) {
        // 利用圆的分割来过滤点云
        std::vector<int> oddNumbers;
        std::vector<int> evenNumbers;
        for (int i = 0; i < path_input->size(); ++i) {
            if (i % 2 == 0) {
                evenNumbers.push_back(i);
            } else {
                oddNumbers.push_back(i);
            }
        }
        // 对奇数和偶数进行从小到大排序
        auto comparator = [](int a, int b) { return a > b; };
        std::sort(oddNumbers.begin(), oddNumbers.end());
        std::sort(evenNumbers.begin(), evenNumbers.end(), comparator);

        std::vector<int> allNumbers;
        allNumbers.insert(allNumbers.end(), oddNumbers.begin(), oddNumbers.end());
        allNumbers.insert(allNumbers.end(), evenNumbers.begin(), evenNumbers.end());

        // 凸包滤波器
        std::vector<pcl::Vertices> polygons;
        pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::Vertices vertics;
        for (const auto& num : allNumbers) {
            vertics.vertices.push_back(num);
        }
        polygons.push_back(vertics);
        surface_hull = path_input;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::CropHull<pcl::PointXYZ> bb_filter;
        bb_filter.setDim(2);
        bb_filter.setInputCloud(cloud_filtered);
        bb_filter.setHullCloud(surface_hull);
        bb_filter.setHullIndices(polygons);
        bb_filter.setNegative(false);
        bb_filter.filter(*cloud_hull);
        printf("filted ok...\n");
        return cloud_hull;
    }

    /*--------------------------------------------------------------------------------*/
   private:
    ros::Subscriber odom_subscriber;
    ros::Publisher trajectory_path_pub;
    ros::Subscriber rslidar_subscriber;
    ros::Publisher trajectory_cloud_pub;

   private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory_point;  // 路径点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rslidar;     // 雷达点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trajectory;  // 障碍点云
};

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "robot_controller");
    // 创建RobotController对象
    RobotController robot_controller;
    // 设置循环的频率为10Hz
    ros::Rate loop_rate(5);  // 设置为1Hz
    while (ros::ok()) {
        // ros::spin();
        // 在循环中处理其他逻辑
        // 循环等待回调函数
        ros::spinOnce();
        // 控制循环的速率
        loop_rate.sleep();
    }
    return 0;
}
