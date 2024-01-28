#include "obscalte_test.h"

double halfWidth;
double halfLength;

namespace trajectory_nm {
Trajectory::Trajectory(ros::NodeHandle nh, tf2_ros::Buffer& tf_Buffer)
    : tf_(tf_Buffer) /*, viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"))*/
{
    ros::NodeHandle nh_param("~");
    nh_param.param<double>("halfLength", halfLength, 0.5);
    nh_param.param<double>("halfWidth", halfWidth, 0.8);
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

/***********************************************************************/
void Trajectory::subPath_callback(const nav_msgs::Path::ConstPtr& msg) {
    // printf("path received ....\n");
    // 保存转换坐标后的路径数据

    this->path_received = *msg;
    this->transformed_points = this->transformPathToBaseLink(msg);
};
void Trajectory::subCloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // printf("cloud received ....\n");
    // 保存转换坐标后的点云数据
    pcl::fromROSMsg(*msg, *this->cloud_received);
};
/***********************************************************************/
std::vector<geometry_msgs::Pose> Trajectory::transformPathToBaseLink(const nav_msgs::Path::ConstPtr& msg) {
    // printf("frame:%s\n", msg->header.frame_id.c_str());
    std::vector<geometry_msgs::Pose> pose;
    for (const auto& pose_stamped : msg->poses) {
        geometry_msgs::PoseStamped pose_stamped_base_link;
        try {
            tf_.transform(pose_stamped, pose_stamped_base_link, "base_footprint", ros::Duration(3));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
        }
        pose.push_back(pose_stamped_base_link.pose);
    }
    // printf("transformed_points size: %d\n", transformed_points.size());
    return pose;
}

/*根据中心点计算滤波矩形四个角点*/
void Trajectory::box_compute(const geometry_msgs::Pose& center, std::vector<BoXPoint>& shift) {
    // printf("box compute.... \n");
    double centerX = center.position.x;
    double centerY = center.position.y;
    double yaw = tf2::getYaw(center.orientation);
    // 计算四个角绝对坐标
    shift.push_back({centerX + halfLength * cos(yaw) - halfWidth * sin(yaw), centerY + halfLength * sin(yaw) + halfWidth * cos(yaw)});
    shift.push_back({centerX - halfLength * cos(yaw) - halfWidth * sin(yaw), centerY - halfLength * sin(yaw) + halfWidth * cos(yaw)});
    shift.push_back({centerX - halfLength * cos(yaw) + halfWidth * sin(yaw), centerY - halfLength * sin(yaw) - halfWidth * cos(yaw)});
    shift.push_back({centerX + halfLength * cos(yaw) + halfWidth * sin(yaw), centerY + halfLength * sin(yaw) - halfWidth * cos(yaw)});
}

double calculateDistance(BoXPoint p1, BoXPoint p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

/*根据路径生成车辆形式的路径*/
void Trajectory::path_calculation(const std::vector<geometry_msgs::Pose>& points) {
    for (const auto& point : points) {
        std::vector<BoXPoint> shift;
        // 根据路径计算角点
        // printf("path calculation.... \n");
        box_compute(point, shift);
        // 计算矩形的对角点
        float Mdistance = 0;
        int index = 0;
        for (int i = 1; i < 4; i++) {
            float distance = calculateDistance(shift.at(0), shift.at(i));
            if (distance > Mdistance) {
                index = i;
                Mdistance = distance;
            }
        }
        // 保存角点
        this->box_shift.push_back({shift.at(0).x, shift.at(0).y, shift.at(index).x, shift.at(index).y, point});
        // printf("x:%2f,y:%2f,x2:%.2f,y2:%.2f,M_dis:%.2f,index:%d\n", shift.at(0).x, shift.at(0).y, shift.at(index).x, shift.at(index).y, Mdistance,
            //    index);
        for (const auto& data : shift) {
            // 保存路径
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
                            float arm_max_y,
                            geometry_msgs::Pose center) {
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
    car_marker.pose.position.z = 0.5;
    car_marker.pose.orientation.x = center.orientation.x;
    car_marker.pose.orientation.y = center.orientation.y;
    car_marker.pose.orientation.z = center.orientation.z;
    car_marker.pose.orientation.w = center.orientation.w;
    // 尺寸
    car_marker.scale.x = halfLength * 2;
    car_marker.scale.y = halfWidth * 2;
    car_marker.scale.z = 0.5;
    // 颜色
    car_marker.color.r = 0.0f;
    car_marker.color.g = 1.0f;
    car_marker.color.b = 0.0f;
    car_marker.color.a = 0.5;
    car_marker.lifetime = ros::Duration(1);
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

void Trajectory::crophull_filter(std::vector<pcl::PointIndices>& point_index) {
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(this->cloud_received);
    crop.setNegative(false);
    // 遍历所有矩形框进行滤波，然后利用聚类算法计算是否存在障碍物
    for (const auto& box_shift_list : this->box_shift) {
        crop.setMin(Eigen::Vector4f(box_shift_list.xmin, box_shift_list.ymin, -0.5, 1.0));
        crop.setMax(Eigen::Vector4f(box_shift_list.xmax, box_shift_list.ymax, 0.5, 1.0));
        crop.filter(*this->cloud_hull_filetered);

        // 如果检测到障碍物，进行聚类算法检测
        if (cloud_hull_filetered->size() > 20) {
            printf("xmax:%.2f, ymax:%.2f, xmin:%.2f, ymin:%.2f\n", box_shift_list.xmax, box_shift_list.ymax, box_shift_list.xmin,
                   box_shift_list.ymin);
            printf("cloud_hull_filetered size:%d\n", this->cloud_hull_filetered->size());

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
                printf("KDTREE size: %d.........\n", point_index.size());
                // 发布滤波使用的角点
                pcl::PointCloud<pcl::PointXYZ>::Ptr pub_shift(new pcl::PointCloud<pcl::PointXYZ>);
                pub_shift->push_back(pcl::PointCloud<pcl::PointXYZ>::PointType(box_shift_list.xmin, box_shift_list.ymin, 0));
                pub_shift->push_back(pcl::PointCloud<pcl::PointXYZ>::PointType(box_shift_list.xmax, box_shift_list.ymax, 0));
                sensor_msgs::PointCloud2 shift_box_ros;
                pcl::toROSMsg(*pub_shift, shift_box_ros);
                shift_box_ros.header.frame_id = "base_footprint";
                shift_box_ros.header.stamp = ros::Time::now();
                filted_point.publish(shift_box_ros);
                pub_shift->clear();
                marker_pub(box_shift_list.xmin, box_shift_list.ymin, box_shift_list.xmax, box_shift_list.ymax, 0, 0, 0, 0, box_shift_list.center);
                break;
            }
        }
    }
    this->box_shift.clear();
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
        // 1. 计算路径
        path_calculation(this->transformed_points);
        // 2. 滤波
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
        float trajectory_min_x = -1;
        for (const auto& data : point_index) {
            float point_x_min = cloud_hull_filetered->points.at(data.indices.at(0)).x;
            for (int index : data.indices) {
                if (cloud_hull_filetered->points.at(index).x < point_x_min) {
                    point_x_min = cloud_hull_filetered->points.at(index).x;
                }
            }
            trajectory_min_x = point_x_min;
        }

        if (trajectory_min_x <= 2.0 && trajectory_min_x > 0.0) {
            cmd_stop.linear.x = 0.0;
            cmd_stop.angular.z = 0.0;
            pubCmd.publish(cmd_stop);
            printf("trajecotry_min_x2.0: %.2f.......\n", trajectory_min_x);
        } else if (trajectory_min_x >= 2.0 && trajectory_min_x < 4.0) {
            pubCmd.publish(cmd_stop);
            printf("trajecotry_min_x4.0: %.2f.......\n", trajectory_min_x);
        }

        cloud_hull_filetered->clear();
        ros::Duration(0.1).sleep();
    };
}

}  // namespace trajectory_nm