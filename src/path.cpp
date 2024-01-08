#include <visualization_msgs/Marker.h>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#define start M_PI / 180.0f * 180.0f
#define end M_PI * 2

class RobotController {
   public:
    RobotController() {
        // 初始化ROS节点
        ros::NodeHandle nh;
        // 创建订阅器，订阅odom消息
        odom_subscriber = nh.subscribe("odom", 1, &RobotController::odomCallback, this);
        // 创建发布器，发布机器人轨迹消息
        // trajectory_publisher = nh.advertise<geometry_msgs::Twist>("robot_trajectory", 10);
        marker_pub_in = nh.advertise<visualization_msgs::Marker>("path_in", 10);
        marker_pub = nh.advertise<visualization_msgs::Marker>("path", 10);
        marker_pub_out = nh.advertise<visualization_msgs::Marker>("path_out", 10);
    }

    // 回调函数处理odom消息
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 提取线速度和角速度
        double linear_velocity = msg->twist.twist.linear.x;
        double angular_velocity = msg->twist.twist.angular.z;
        if (abs(linear_velocity) >= 0.002)
            // 生成机器人轨迹曲线
            generateTrajectory(linear_velocity, angular_velocity);
    }

    // 生成机器人轨迹曲线的示例函数
    void generateTrajectory(double linear_velocity, double angular_velocity) {
        // 计算转弯半径
        double turning_radius = linear_velocity / angular_velocity;
        // 生成机器人轨迹曲线
        publisherMaerker(turning_radius, 0.5f);
        // 打印转弯半径
        ROS_INFO("Turning Radius: %.2f", turning_radius);
    }
    // 发布机器人轨迹信息
    void publisherMaerker(float r, float width) {
        if (r >= 0) {
            width = abs(width);
        } else {
            width = -abs(width);
        }

        visualization_msgs::Marker marker;
        visualization_msgs::Marker marker_in;
        visualization_msgs::Marker marker_out;
        marker.header.frame_id = "base_link";  // 设置坐标系，根据你的实际情况修改
        marker.header.stamp = ros::Time::now();
        marker.ns = "circle_trajectory";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        // 设置标记类型为线条
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        // 设置标记的尺寸和颜色
        marker.scale.x = 0.05;  // 线条宽度
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_in = marker;
        marker_out = marker;
        // 直线行驶
        if (abs(r) > 30.0) {
            geometry_msgs::Point point;
            point.x = 3.0;
            point.y = 0.0;
            point.z = 0.0;
            marker.points.push_back(point);
            point.x = -3.0;
            marker.points.push_back(point);

            point.x = 3.0;
            point.y = width;
            marker_out.points.push_back(point);
            point.x = -3.0;
            marker_out.points.push_back(point);

            point.x = 3.0;
            point.y = -width;
            marker_in.points.push_back(point);
            point.x = -3.0;
            marker_in.points.push_back(point);
        } else {
            // 曲线行驶
            for (double theta = start; theta <= end; theta += 0.1) {
                geometry_msgs::Point point;
                // 内圆
                point.x = cos(theta) * (r - width);
                point.y = sin(theta) * (r - width) + r;
                point.z = 0.0;
                if (point.x <= 3.0 && point.x >= 0)
                    marker_in.points.push_back(point);
                // 中心圆
                point.x = cos(theta) * r;
                point.y = sin(theta) * r + r;
                if (point.x <= 3.0 && point.x >= 0)
                    marker.points.push_back(point);
                // 外圆
                point.x = cos(theta) * (r + width);
                point.y = sin(theta) * (r + width) + r;
                if (point.x <= 3.0 && point.x >= 0)
                    marker_out.points.push_back(point);
            }
        }

        marker_pub.publish(marker);
        marker_pub_in.publish(marker_in);
        marker_pub_out.publish(marker_out);
    }

   private:
    ros::Subscriber odom_subscriber;
    // ros::Publisher trajectory_publisher;
    ros::Publisher marker_pub;
    ros::Publisher marker_pub_in;
    ros::Publisher marker_pub_out;
    // geometry_msgs::Twist trajectory;
};

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "robot_controller");
    // 创建RobotController对象
    RobotController robot_controller;
    // 设置循环的频率为10Hz
    ros::Rate loop_rate(10);  // 设置为1Hz
    while (ros::ok()) {
        // 在循环中处理其他逻辑
        // ...

        // 循环等待回调函数
        ros::spinOnce();

        // 控制循环的速率
        loop_rate.sleep();
    }
    return 0;
}
