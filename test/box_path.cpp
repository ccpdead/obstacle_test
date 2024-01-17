#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    // yaw 45 deg
    geometry_msgs::Pose center;
    center.position.x = 2;
    center.position.y = 2;
    center.position.z = 0;
    center.orientation.w = 0.9238795;
    center.orientation.x = 0;
    center.orientation.y = 0;
    center.orientation.z = 0.3826834;

    double centerX = center.position.x;
    double centerY = center.position.y;

    struct Point {
        double x;
        double y;
    };

    std::vector<Point> corners;
    double yaw = tf::getYaw(center.orientation);

    double halfWidth = 0.4;
    double halfLength = 0.8;

    // 计算四个角绝对坐标
    corners.push_back({centerX + halfWidth * cos(yaw) - halfLength * sin(yaw), centerY + halfWidth * sin(yaw) + halfLength * cos(yaw)});
    corners.push_back({centerX - halfWidth * cos(yaw) - halfLength * sin(yaw), centerY - halfWidth * sin(yaw) + halfLength * cos(yaw)});
    corners.push_back({centerX - halfWidth * cos(yaw) + halfLength * sin(yaw), centerY - halfWidth * sin(yaw) - halfLength * cos(yaw)});
    corners.push_back({centerX + halfWidth * cos(yaw) + halfLength * sin(yaw), centerY + halfWidth * sin(yaw) - halfLength * cos(yaw)});

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& data : corners) {
        cloud->push_back(pcl::PointXYZ(data.x, data.y, 0));
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> for_visualizer_v(new pcl::visualization::PCLVisualizer("crophull display"));
    for_visualizer_v->setBackgroundColor(255, 255, 255);
    int v1(0);  // 显示原始点云
    for_visualizer_v->createViewPort(0.0, 0.0, 1, 1, v1);
    for_visualizer_v->setBackgroundColor(0, 0, 0, v1);
    for_visualizer_v->addPointCloud(cloud, "cloud", v1);
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "cloud");
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    for_visualizer_v->addPolygon<pcl::PointXYZ>(cloud, 0, .069 * 255, 0.2 * 255, "backview_hull_polyline1", v1);

    // 画出四个角
    for (const auto& data : corners) {
        std::cout << data.x << " " << data.y << std::endl;
    }
    // 添加pcl坐标轴显示
    for_visualizer_v->addCoordinateSystem(1.0);
    for_visualizer_v->initCameraParameters();
    while (!for_visualizer_v->wasStopped()) {
        for_visualizer_v->spinOnce(1000);
    }
}