#include "trajectory.h"

namespace trajectory {

Trajectory::Trajectory(ros::NodeHandle nh, tf2_ros::Buffer& tf_)
    :viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer")) {
    subPath = nh.subscribe("/move_base/DWAPlannerROS/local_plan", 1, &Trajectory::subPath_callback, this);
    subCloud = nh.subscribe("/velodyne_points", 1, &Trajectory::subCloud_callback, this);

    init_data();

    std::thread t1(&Trajectory::process, this);
    t1.detach();

    std::thread t2(&Trajectory::view_point, this);
    t2.detach();
};

Trajectory::~Trajectory(){};
void Trajectory::subPath_callback(const nav_msgs::Path::ConstPtr& msg) {
    // 保存转换坐标后的路径数据
    this->path_received = *msg;
};

void Trajectory::subCloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 保存转换坐标后的点云数据
    pcl::fromROSMsg(*msg, *this->cloud_currnet);
};

/*tf转换*/
std::vector<geometry_msgs::Point> Trajectory::transformPathToBaseLink(const nav_msgs::Path::ConstPtr& msg) {
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

/*根据路径生成车辆形式的路径*/
void Trajectory::path_calculation(const std::vector<geometry_msgs::Point>& points) {
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

    // 分割奇数偶数
    std::vector<int> oddNumbers;
    std::vector<int> evenNumbers;
    for (int i = 0; i < cloud_pcl->size(); ++i) {
        if (i % 2 == 0) {
            evenNumbers.push_back(i);
        } else {
            oddNumbers.push_back(i);
        }
    }
    // 从新排列
    auto comparator = [](int a, int b) { return a > b; };
    std::sort(oddNumbers.begin(), oddNumbers.end());
    std::sort(evenNumbers.begin(), evenNumbers.end(), comparator);
    std::vector<int> allNumbers;
    allNumbers.insert(allNumbers.end(), oddNumbers.begin(), oddNumbers.end());
    allNumbers.insert(allNumbers.end(), evenNumbers.begin(), evenNumbers.end());

    for (const auto& num : allNumbers) {
        this->cloud_path->push_back(cloud_pcl->points[num]);
    }

    this->polygons = allNumbers;
    this->surface_hull = cloud_pcl;
}

void Trajectory::crophull_filter() {
    // 凸包滤波器
    std::vector<pcl::Vertices> polygons;
    pcl::Vertices vertics;
    for (const auto& num : this->polygons) {
        vertics.vertices.push_back(num);
    }
    polygons.push_back(vertics);
    pcl::CropHull<pcl::PointXYZ> polygon_extract;
    polygon_extract.setDim(2);
    polygon_extract.setInputCloud(this->cloud_currnet);
    polygon_extract.setHullCloud(this->surface_hull);
    polygon_extract.setHullIndices(polygons);
    polygon_extract.setNegative(false);
    polygon_extract.filter(*this->cloud_hull_filetered);
    printf("filetered ok .....\n");
}

void Trajectory::view_point() {
    std::string cloud_id = "cloud_" + std::to_string(ros::Time::now().toSec());
    int v2(0);  // 显示封闭2D多边形凸包
    viewer->createViewPort(0.0, 0.0, 1, 1, v2);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->addPointCloud(this->cloud_path, cloud_id, v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, cloud_id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, cloud_id);
    viewer->addPolygon<pcl::PointXYZ>(this->cloud_path, 0, .069 * 255, 0.2 * 255, cloud_id, v2);
    viewer->spinOnce(10);
}

void Trajectory::init_data() {
    this->car_width = 0.4f;
}

void Trajectory::process() {
    // 1. 等待数据
    while (ros::ok()) {
        while (cloud_currnet->empty() && path_received.poses.empty()) {
            std::cout << "waiting for data" << std::endl;
            ros::Duration(1).sleep();
        }
        // tf to base_link
        std::vector<geometry_msgs::Point> transformed_points = transformPathToBaseLink(boost::shared_ptr<nav_msgs::Path>(&path_received));
        // 2. 路径点云生成
        path_calculation(transformed_points);
    };
}

}  // namespace trajectory