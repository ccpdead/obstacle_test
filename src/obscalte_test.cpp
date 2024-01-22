#include "obscalte_test.h"

double halfWidth;
double halfLength;

namespace trajectory_nm {
Trajectory::Trajectory(ros::NodeHandle nh, tf2_ros::Buffer& tf_Buffer)
    : tf_(tf_Buffer) /*, viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"))*/
{
    ros::NodeHandle nh_param("~");
    nh_param.param<double>("halfLength", halfLength, 1.3);
    nh_param.param<double>("halfWidth", halfWidth, 0.68);
    subPath = nh.subscribe("/move_base/local_plan", 1, &Trajectory::subPath_callback, this);
    subCloud = nh.subscribe("/lidar_fusion", 1, &Trajectory::subCloud_callback, this);

    pubPath = nh.advertise<sensor_msgs::PointCloud2>("car_path", 1);    // 路径信息
    pubCloud = nh.advertise<sensor_msgs::PointCloud2>("car_cloud", 1);  // 车辆滤波角点
    pubCmd = nh.advertise<geometry_msgs::Twist>("cmd_vel_stop", 1);
    car_marker_pub = nh.advertise<visualization_msgs::Marker>("car_marker", 1);
    arm_marker_pub = nh.advertise<visualization_msgs::Marker>("arm_marker", 1);
    filted_point = nh.advertise<sensor_msgs::PointCloud2>("filted_point", 1);  // 滤波后检测到的点云

    init_data();
    std::thread t1(&Trajectory::process, this);
    t1.detach();
};

Trajectory::~Trajectory(){};
void Trajectory::subPath_callback(const nav_msgs::Path::ConstPtr& msg) {
    // 保存转换坐标后的路径数据
    this->path_received = *msg;
    transformed_points = this->transformPathToBaseLink(msg);
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
            tf_.transform(pose_stamped, pose_stamped_base_link, "base_footprint", ros::Duration(3));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
        }
        transformed_points.push_back(pose_stamped_base_link.pose);
    }
    // printf("transformed_points size: %d\n", transformed_points.size());
    return transformed_points;
}

/*根据中心点计算滤波矩形*/
void Trajectory::box_compute(const geometry_msgs::Pose& center, std::vector<BoXPoint>& shift) {
    double centerX = center.position.x;
    double centerY = center.position.y;
    double yaw = tf2::getYaw(center.orientation);
    // 计算四个角绝对坐标
    shift.push_back({centerX + halfWidth * cos(yaw) - halfLength * sin(yaw), centerY + halfWidth * sin(yaw) + halfLength * cos(yaw)});
    shift.push_back({centerX - halfWidth * cos(yaw) - halfLength * sin(yaw), centerY - halfWidth * sin(yaw) + halfLength * cos(yaw)});
    shift.push_back({centerX - halfWidth * cos(yaw) + halfLength * sin(yaw), centerY - halfWidth * sin(yaw) - halfLength * cos(yaw)});
    shift.push_back({centerX + halfWidth * cos(yaw) + halfLength * sin(yaw), centerY + halfWidth * sin(yaw) - halfLength * cos(yaw)});
}

/*根据路径生成车辆形式的路径*/
void Trajectory::path_calculation(const std::vector<geometry_msgs::Pose>& points) {
    // printf("path path_calculation\n");
    for (const auto& point : points) {
        std::vector<BoXPoint> shift;
        // 根据路径计算角点
        box_compute(point, shift);
        // 计算矩形的最大最小值
        float xmin, ymin, xmax, ymax;
        xmin = ymin = shift.at(0).x;
        xmax = ymax = shift.at(0).y;
        for (const auto& box_shift_list : shift) {
            if (box_shift_list.x < xmin)
                xmin = box_shift_list.x;
            if (box_shift_list.y < ymin)
                ymin = box_shift_list.y;
            if (box_shift_list.x > xmax)
                xmax = box_shift_list.x;
            if (box_shift_list.y > ymax)
                ymax = box_shift_list.y;
        }
        this->box_shift.push_back({xmin, xmax, ymin, ymax});
        // printf("xmin:%.2f, xmax:%.2f, ymin:%.2f, ymax:%.2f\n", xmin, xmax, ymin, ymax);
        for (const auto& data : shift) {
            this->cloud_path->push_back(pcl::PointXYZ(data.x, data.y, 0));
        }
    }
}

/**
 * 当检测到障碍物后，将车身等比例的marker发布
 */
void Trajectory::marker_pub(float car_min_x,
                            float car_min_y,
                            float car_max_x,
                            float car_max_y,
                            float arm_min_x,
                            float arm_max_x,
                            float arm_min_y,
                            float arm_max_y) {
    visualization_msgs::Marker car_marker;
    visualization_msgs::Marker arm_marker;

    car_marker.header.frame_id = "base_footprint";
    car_marker.header.stamp = ros::Time::now();
    car_marker.ns = "car";
    car_marker.id = 0;
    car_marker.type = visualization_msgs::Marker::CUBE;
    car_marker.action = visualization_msgs::Marker::ADD;
    // 位置
    car_marker.pose.position.x = (car_min_x + car_max_x) / 2;
    car_marker.pose.position.y = (car_min_y + car_max_y) / 2;
    car_marker.pose.position.z = 0.0;
    car_marker.pose.orientation.x = 0.0;
    car_marker.pose.orientation.y = 0.0;
    car_marker.pose.orientation.z = 0.0;
    car_marker.pose.orientation.w = 1.0;
    // 尺寸
    car_marker.scale.x = halfWidth * 2;
    car_marker.scale.y = halfLength * 2;
    car_marker.scale.z = 0.5;
    // 颜色
    car_marker.color.r = 0.0f;
    car_marker.color.g = 1.0f;
    car_marker.color.b = 0.0f;
    car_marker.color.a = 0.5;
    car_marker.lifetime = ros::Duration(0);
    car_marker_pub.publish(car_marker);  // 发布marker
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
void Trajectory::crophull_filter(std::vector<pcl::PointIndices>& point_index) {
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(this->cloud_received);
    crop.setNegative(false);
    // 遍历所有矩形框进行滤波，然后利用聚类算法计算是否存在障碍物
    for (const auto& box_shift_list : this->box_shift) {
        crop.setMin(Eigen::Vector4f(box_shift_list.xmin, box_shift_list.ymin, -0.5, 1.0));
        crop.setMax(Eigen::Vector4f(box_shift_list.xmax, box_shift_list.ymax, 0.5, 1.0));
        crop.filter(*this->cloud_hull_filetered);
        printf("xmin:%.2f, xmax:%.2f, ymin:%.2f, ymax:%.2f\n", box_shift_list.xmin, box_shift_list.xmax, box_shift_list.ymin, box_shift_list.ymax);
        printf("cloud_hull_filetered size:%d\n", this->cloud_hull_filetered->size());

        // 发布滤波使用的角点
        pcl::PointCloud<pcl::PointXYZ>::Ptr filte_point_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        filte_point_pcl->push_back(pcl::PointCloud<pcl::PointXYZ>::PointType(box_shift_list.xmin, box_shift_list.ymin, 0));
        filte_point_pcl->push_back(pcl::PointCloud<pcl::PointXYZ>::PointType(box_shift_list.xmax, box_shift_list.ymax, 0));
        sensor_msgs::PointCloud2 filte_point_ros;
        pcl::toROSMsg(*filte_point_pcl, filte_point_ros);
        filte_point_ros.header.frame_id = "base_footprint";
        filte_point_ros.header.stamp = ros::Time::now();

        if (cloud_hull_filetered->size() > 10) {
            // 聚类算法检测障碍物数量
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud_hull_filetered);
            // 创建euclidean聚类对象
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.2);  // 设置近邻搜索的搜索半径为20cm
            ec.setMinClusterSize(20);     // 设置一个聚类需要的最少的点数目为100
            ec.setMaxClusterSize(9999);   // 设置一个聚类需要的最大点数目为25000
            ec.setSearchMethod(tree);     // 设置点云的搜索机制
            ec.setInputCloud(cloud_hull_filetered);
            // std::vector<pcl::PointIndices> clustr_indices;
            ec.extract(point_index);
            if (point_index.size() > 0) {
                filted_point.publish(filte_point_ros);
                marker_pub(box_shift_list.xmin, box_shift_list.ymin, box_shift_list.xmax, box_shift_list.ymax, 0, 0, 0, 0);
                printf("KDTREE size: %d.........\n", point_index.size());
                break;
            }
        }
    }
}
/*初始化数据*/
void Trajectory::init_data() {
    this->cloud_received.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->path_current.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->cloud_hull_filetered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->cloud_path.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->surface_hull.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

// 多线程处理函数
void Trajectory::process() {
    geometry_msgs::Twist cmd_stop;
    // 1. 等待数据
    while (ros::ok()) {
        while (cloud_received->empty() && path_received.poses.empty()) {
            std::cout << "waiting for data" << std::endl;
            ros::Duration(1).sleep();
        }
        std::vector<pcl::PointIndices> point_index;
        path_calculation(this->transformed_points);
        crophull_filter(point_index);

        // 发布路径
        sensor_msgs::PointCloud2 path_cloud;
        pcl::toROSMsg(*this->cloud_path, path_cloud);
        path_cloud.header.frame_id = "base_footprint";
        path_cloud.header.stamp = ros::Time::now();
        pubPath.publish(path_cloud);
        cloud_path->clear();
        // 发布滤波点云
        sensor_msgs::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(*this->cloud_hull_filetered, point_cloud_msg);
        point_cloud_msg.header.frame_id = "base_footprint";
        point_cloud_msg.header.stamp = ros::Time::now();
        pubCloud.publish(point_cloud_msg);

        // 计算最近点云距离
        float trajectory_min_x = 0.0;
        for (const auto& data : point_index) {
            float point_x_min = cloud_hull_filetered->points.at(data.indices.at(0)).x;
            for (int index : data.indices) {
                if (cloud_hull_filetered->points.at(index).x < point_x_min) {
                    point_x_min = cloud_hull_filetered->points.at(index).x;
                }
            }
            trajectory_min_x = point_x_min;
        }

        printf("trajecotry_min_x: %.2f.......\n", trajectory_min_x);
        if (trajectory_min_x < 1.7) {
            cmd_stop.linear.x = 0.0;
            pubCmd.publish(cmd_stop);
        } else if (trajectory_min_x >= 1.7 && trajectory_min_x < 3.5) {
            cmd_stop.linear.x = 0.05;
            pubCmd.publish(cmd_stop);
        }

        cloud_hull_filetered->clear();
        ros::Duration(0.2).sleep();
    };
}

}  // namespace trajectory_nm