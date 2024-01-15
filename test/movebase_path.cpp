/**
 * 1. 读取movebase的路径，并将此从map坐标系下转化为base_link坐标系下的路径
 * 2. 平移路径，并检测路径内的点云障碍物
 */

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <vector>

#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

class movebase_path {
   public:
    movebase_path(ros::NodeHandle &nh) {
        subPath = nh.subscribe("/move_base/local_plan", 1, &movebase_path::local_plan_callback, this);
        pubPath = nh.advertise<sensor_msgs::PointCloud2>("car_path", 1);
    };
    ~movebase_path(){};

   public:
    void local_plan_callback(const nav_msgs::Path::ConstPtr& msg) {
        tf2_ros::Buffer tfBuffer;  // 使用tf进行坐标转换
        tf2_ros::TransformListener tfListener(tfBuffer);

        // 保存转换坐标后的路径数据
        std::vector<geometry_msgs::Point> transformed_points = transformPathToBaseLink(msg, tfBuffer);

        publishPointCloud(transformed_points);
    }

    std::vector<geometry_msgs::Point> transformPathToBaseLink(const nav_msgs::Path::ConstPtr& msg, tf2_ros::Buffer& tf_buffer) {
        std::vector<geometry_msgs::Point> transformed_points;
        for (const auto& pose_stamped : msg->poses) {
            geometry_msgs::PoseStamped pose_stamped_base_link;
            try {
                tf_buffer.transform(pose_stamped, pose_stamped_base_link, "base_link");
            } catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                continue;
            }
            transformed_points.push_back(pose_stamped_base_link.pose.position);
        }

        return transformed_points;
    }

    void publishPointCloud(const std::vector<geometry_msgs::Point>& points) {
        sensor_msgs::PointCloud2 point_cloud_msg;
        point_cloud_msg.header.frame_id = "base_link";
        point_cloud_msg.header.stamp = ros::Time::now();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& data : points) {
            pcl::PointXYZ point;
            pcl::PointXYZ point_left;
            pcl::PointXYZ point_right;
            point.x = data.x;
            point.y = data.y;
            point.z = data.z;

            point_left = point_right = point;
            point_left.x = point.x - 0.2;
            point_right.x = point.x + 0.2;

            cloud_pcl->push_back(point);
            cloud_pcl->push_back(point_left);
            cloud_pcl->push_back(point_right);
        }

        pcl::toROSMsg(*cloud_pcl, point_cloud_msg);
        pubPath.publish(point_cloud_msg);
    }

   private:
    nav_msgs::Path::ConstPtr path;

    ros::Subscriber subPath;  // 订阅路径
    ros::Publisher pubPath;   // 发布路径
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "movebase_path");
    ros::NodeHandle nh;
    movebase_path path_test(nh);
    ros::spin();
}
