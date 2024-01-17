/**
 * 1. 读取movebase的路径，并将此从map坐标系下转化为base_link坐标系下的路径
 * 2. 平移路径，并检测路径内的点云障碍物
 */
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <iostream>
#include <vector>

#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

float car_width = 0.08f;

class movebase_path {
   public:
    movebase_path(tf2_ros::Buffer& tf_Buffer) : tf_(tf_Buffer) ,viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer")) {
        ros::NodeHandle nh;
        subPath = nh.subscribe("/move_base/DWAPlannerROS/local_plan", 1, &movebase_path::local_plan_callback, this);
        pubPath = nh.advertise<sensor_msgs::PointCloud2>("car_path", 1);
    };
    ~movebase_path(){};

   public:
    void local_plan_callback(const nav_msgs::Path::ConstPtr& msg) {
        // 保存转换坐标后的路径数据
        std::vector<geometry_msgs::Point> transformed_points = transformPathToBaseLink(msg);
        publishPointCloud(transformed_points);
    }
    /*tf转换*/
    std::vector<geometry_msgs::Point> transformPathToBaseLink(const nav_msgs::Path::ConstPtr& msg) {
        std::vector<geometry_msgs::Point> transformed_points;
        for (const auto& pose_stamped : msg->poses) {
            geometry_msgs::PoseStamped pose_stamped_base_link;
            try {
                tf_.transform(pose_stamped, pose_stamped_base_link, "base_link", ros::Duration(3));
            } catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
            }
            transformed_points.push_back(pose_stamped_base_link.pose.position);
        }

        return transformed_points;
    }
    
    void publishPointCloud(const std::vector<geometry_msgs::Point>& points) {
        sensor_msgs::PointCloud2 point_cloud_msg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& data : points) {
            pcl::PointXYZ point;
            pcl::PointXYZ point_l;
            pcl::PointXYZ point_r;
            point.x = data.x;
            point.y = data.y;
            point.z = data.z;

            point_l = point_r = point;
            point_l.y = point_l.y - car_width;
            point_r.y = point_r.y + car_width;
            cloud_pcl->push_back(point_l);
            cloud_pcl->push_back(point_r);
        }
        // cloud_view(cloud_pcl);
        cloud_soring(cloud_pcl);
        pcl::toROSMsg(*cloud_pcl, point_cloud_msg);
        point_cloud_msg.header.frame_id = "base_link";
        point_cloud_msg.header.stamp = ros::Time::now();
        pubPath.publish(point_cloud_msg);
    }

    void cloud_soring(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud) {
        // 分割奇数偶数
        std::vector<int> oddNumbers;
        std::vector<int> evenNumbers;
        for (int i = 0; i < input_cloud->size(); ++i) {
            if (i % 2 == 0) {
                evenNumbers.push_back(i);
            } else {
                oddNumbers.push_back(i);
            }
        }
        // 奇偶从新排列
        auto comparator = [](int a, int b) { return a > b; };
        std::sort(oddNumbers.begin(), oddNumbers.end());
        std::sort(evenNumbers.begin(), evenNumbers.end(), comparator);
        std::vector<int> allNumbers;
        allNumbers.insert(allNumbers.end(), oddNumbers.begin(), oddNumbers.end());
        allNumbers.insert(allNumbers.end(), evenNumbers.begin(), evenNumbers.end());

        // // 输出所有数
        // for (const auto& num : allNumbers) {
        //     std::cout << num << " ";
        // }
        // std::cout << std::endl;
        // // 输出奇数
        // std::cout << "oddNumbers:" << std::endl;
        // for (const auto& num : oddNumbers) {
        //     std::cout << num << " ";
        // }
        // std::cout << std::endl;
        // // 输出偶数
        // std::cout << "evenNumbers:" << std::endl;
        // for (const auto& num : evenNumbers) {
        //     std::cout << num << " ";
        // }
        // std::cout << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr view_point(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& i : allNumbers) {
            view_point->push_back(input_cloud->points[i]);
        }
        cloud_view(view_point);
        view_point.unique();
    }

    void point_filter() {}
    void cloud_view(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_view) {
        printf("size: %d\n", cloud_view->size());

        // static pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
        std::string cloud_id = "cloud_" + std::to_string(ros::Time::now().toSec());
        int v2(0);  // 显示封闭2D多边形凸包
        viewer->createViewPort(0.0, 0.0, 1, 1, v2);
        viewer->setBackgroundColor(0, 0, 0, v2);
        viewer->addPointCloud(cloud_view, cloud_id, v2);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, cloud_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, cloud_id);
        viewer->addPolygon<pcl::PointXYZ>(cloud_view, 0, .069 * 255, 0.2 * 255, cloud_id, v2);
        viewer->spinOnce(10);
    }

   private:
    ros::Subscriber subPath;  // 订阅路径
    ros::Publisher pubPath;   // 发布路径
    tf2_ros::Buffer& tf_;     // tf2_ros::Buffer的引用
    pcl::PointCloud<pcl::PointXYZ>::Ptr view_point;
    pcl::visualization::PCLVisualizer::Ptr viewer;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "movebase_path");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;                         // 使用tf进行坐标转换
    tf2_ros::TransformListener tfListener(tfBuffer);  // 启动监听
    movebase_path path_test(tfBuffer);
    ros::spin();
}
