#include "trajectory.h"

namespace trajectory_nm {

Trajectory::Trajectory(ros::NodeHandle nh, tf2_ros::Buffer& tf_Buffer)
    : tf_(tf_Buffer) /*, viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"))*/
{
    subPath = nh.subscribe("/move_base/DWAPlannerROS/local_plan", 1, &Trajectory::subPath_callback, this);
    subCloud = nh.subscribe("/lidar_fusion", 1, &Trajectory::subCloud_callback, this);

    pubPath = nh.advertise<sensor_msgs::PointCloud2>("car_path", 1);
    pubCloud = nh.advertise<sensor_msgs::PointCloud2>("car_cloud", 1);
    pubCmd = nh.advertise<geometry_msgs::Twist>("cmd_stop", 1);

    init_data();
    std::thread t1(&Trajectory::process, this);
    t1.detach();
};

Trajectory::~Trajectory(){};
void Trajectory::subPath_callback(const nav_msgs::Path::ConstPtr& msg) {
    // 保存转换坐标后的路径数据
    this->path_received = *msg;
    transformed_points = this->transformPathToBaseLink(msg);

    /*test*/
    // view_point();
};

void Trajectory::subCloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 保存转换坐标后的点云数据
    pcl::fromROSMsg(*msg, *this->cloud_received);
};

/*路径点tf转换*/
std::vector<geometry_msgs::Pose> Trajectory::transformPathToBaseLink(const nav_msgs::Path::ConstPtr& msg) {
    std::vector<geometry_msgs::Pose> transformed_points;
    for (const auto& pose_stamped : msg->poses) {
        geometry_msgs::PoseStamped pose_stamped_base_link;
        try {
            tf_.transform(pose_stamped, pose_stamped_base_link, "base_link", ros::Duration(3));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
        }
        transformed_points.push_back(pose_stamped_base_link.pose);
    }

    printf("transformed_points size: %d\n", transformed_points.size());
    return transformed_points;
}

/*根据中心点计算滤波矩形*/
void Trajectory::box_compute(const geometry_msgs::Pose& center, std::vector<BoXPoint>& shift) {
    double centerX = center.position.x;
    double centerY = center.position.y;

    double yaw = tf2::getYaw(center.orientation);

    double halfWidth = 0.04;
    double halfLength = 0.07;

    // 计算四个角绝对坐标
    shift.push_back({centerX + halfWidth * cos(yaw) - halfLength * sin(yaw), centerY + halfWidth * sin(yaw) + halfLength * cos(yaw)});
    shift.push_back({centerX - halfWidth * cos(yaw) - halfLength * sin(yaw), centerY - halfWidth * sin(yaw) + halfLength * cos(yaw)});
    shift.push_back({centerX - halfWidth * cos(yaw) + halfLength * sin(yaw), centerY - halfWidth * sin(yaw) - halfLength * cos(yaw)});
    shift.push_back({centerX + halfWidth * cos(yaw) + halfLength * sin(yaw), centerY + halfWidth * sin(yaw) - halfLength * cos(yaw)});
}

/*根据路径生成车辆形式的路径*/
void Trajectory::path_calculation(const std::vector<geometry_msgs::Pose>& points) {
    printf("path path_calculation\n");
    for (const auto& point : points) {
        std::vector<BoXPoint> box_point;
        // 根据路径计算角点
        box_compute(point, box_point);

        // 计算矩形的最大最小值
        float xmin, ymin, xmax, ymax;
        xmin = ymin = box_point.at(0).x;
        xmax = ymax = box_point.at(0).y;
        for (const auto& box_shift : box_point) {
            if (box_shift.x < xmin)
                xmin = box_shift.x;
            if (box_shift.y < ymin)
                ymin = box_shift.y;
            if (box_shift.x > xmax)
                xmax = box_shift.x;
            if (box_shift.y > ymax)
                ymax = box_shift.y;
        }

        box_shift.push_back({xmin, xmax, ymin, ymax});
        printf("xmin:%.2f, xmax:%.2f, ymin:%.2f, ymax:%.2f\n", xmin, xmax, ymin, ymax);
        for (const auto& data : box_point) {
            this->cloud_path->push_back(pcl::PointXYZ(data.x, data.y, 0));
        }
    }
}
// void Trajectory::view_point() {
//     std::string cloud_id = "cloud_" + std::to_string(ros::Time::now().toSec());
//     int v2(0);  // 显示封闭2D多边形凸包
//     viewer->createViewPort(0.0, 0.0, 1, 1, v2);
//     viewer->setBackgroundColor(0, 0, 0, v2);
//     viewer->addPointCloud(this->cloud_path, cloud_id, v2);
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, cloud_id);
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, cloud_id);
//     viewer->addPolygon<pcl::PointXYZ>(this->cloud_path, 0, .069 * 255, 0.2 * 255, cloud_id, v2);
//     viewer->spinOnce(0.1);
//     printf("cloud_path size:%d\n", this->cloud_path->size());
// }
/*滤波器*/
void Trajectory::crophull_filter() {
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(this->cloud_received);
    crop.setNegative(false);
    for (const auto& box_shift : box_shift) {
        crop.setMin(Eigen::Vector4f(box_shift.xmin, box_shift.ymin, -0.5, 1.0));
        crop.setMax(Eigen::Vector4f(box_shift.xmax, box_shift.ymax, 0.5, 1.0));
        crop.filter(*this->cloud_hull_filetered);
        if (cloud_hull_filetered->size() > 0)
            break;
    }
    printf("cloud_hull_filetered size:%d\n", this->cloud_hull_filetered->size());
}
/*初始化数据*/
void Trajectory::init_data() {
    this->car_width = 0.08f;
    this->cloud_received.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->path_current.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->cloud_hull_filetered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->cloud_path.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->surface_hull.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void Trajectory::process() {
    // 1. 等待数据
    while (ros::ok()) {
        while (cloud_received->empty() && path_received.poses.empty()) {
            std::cout << "waiting for data" << std::endl;
            ros::Duration(1).sleep();
        }
        path_calculation(this->transformed_points);  // 路径生成
        crophull_filter();                           // 点云滤波

        // 发布路径
        sensor_msgs::PointCloud2 path_cloud;
        pcl::toROSMsg(*this->cloud_path, path_cloud);
        path_cloud.header.frame_id = "base_link";
        path_cloud.header.stamp = ros::Time::now();
        pubPath.publish(path_cloud);
        cloud_path->clear();
        // // 发布滤波点云
        // sensor_msgs::PointCloud2 point_cloud_msg;
        // pcl::toROSMsg(*this->cloud_hull_filetered, point_cloud_msg);
        // point_cloud_msg.header.frame_id = "base_link";
        // point_cloud_msg.header.stamp = ros::Time::now();
        // pubCloud.publish(point_cloud_msg);

        // geometry_msgs::Twist cmd_stop;
        // cmd_stop.linear.x = 0.25;

        // // 打印平均距离
        // if (cloud_hull_filetered->points.size() > 20) {
        //     float min_x = cloud_hull_filetered->points.at(1).x;
        //     for (const auto& data : cloud_hull_filetered->points) {
        //         if (data.x < min_x) {
        //             min_x = data.x;
        //         }
        //     }
        //     printf("trajectory x distance: ---%f---\n", min_x);
        //     if (min_x < 1)
        //         cmd_stop.linear.x = 0.0;
        //     else if (min_x > 1 && min_x < 3)
        //         cmd_stop.linear.x = 0.5;
        // }

        // cloud_hull_filetered->clear();
        // pubCmd.publish(cmd_stop);

        // view_point();
        ros::Duration(0.5).sleep();
    };
}

}  // namespace trajectory_nm