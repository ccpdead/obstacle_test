#include "trajectory.h"

namespace trajectory_nm {

Trajectory::Trajectory(ros::NodeHandle nh, tf2_ros::Buffer& tf_Buffer)
    : tf_(tf_Buffer), viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer")) {
    subPath = nh.subscribe("/move_base/local_plan", 1, &Trajectory::subPath_callback, this);
    subCloud = nh.subscribe("/lidar_fusion", 1, &Trajectory::subCloud_callback, this);

    pubPath = nh.advertise<sensor_msgs::PointCloud2>("car_path", 1);
    pubCloud = nh.advertise<sensor_msgs::PointCloud2>("car_cloud", 1);

    init_data();

    std::thread t1(&Trajectory::process, this);
    t1.detach();
};

Trajectory::~Trajectory(){};
void Trajectory::subPath_callback(const nav_msgs::Path::ConstPtr& msg) {
    // 保存转换坐标后的路径数据
    this->path_received = *msg;
    transformed_points = this->transformPathToBaseLink(msg);
    // view_point();
};

void Trajectory::subCloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 保存转换坐标后的点云数据
    pcl::fromROSMsg(*msg, *this->cloud_currnet);
};

/*tf转换*/
std::vector<geometry_msgs::Point> Trajectory::transformPathToBaseLink(const nav_msgs::Path::ConstPtr& msg) {
    std::vector<geometry_msgs::Point> transformed_points;
    for (const auto& pose_stamped : msg->poses) {
        // printf("befor\n");
        // printf("path x:%f\n", pose_stamped.pose.position.x);
        // printf("path y:%f\n", pose_stamped.pose.position.y);
        // printf("path z:%f\n", pose_stamped.pose.position.z);

        geometry_msgs::PoseStamped pose_stamped_base_link;
        try {
            tf_.transform(pose_stamped, pose_stamped_base_link, "lidar_link", ros::Duration(3));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            // continue;
        }
        transformed_points.push_back(pose_stamped_base_link.pose.position);
    }
    // printf("after\n");
    // for (const auto& data : transformed_points) {
    //     printf("x:%f\n", data.x);
    //     printf("y:%f\n", data.y);
    //     printf("z:%f\n", data.z);
    // }

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
    // printf("filetered ok .....\n");
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
    viewer->spinOnce(0.1);
    printf("cloud_path size:%d\n", this->cloud_path->size());
}

void Trajectory::init_data() {
    this->car_width = 0.04f;

    this->cloud_currnet.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->path_current.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->cloud_hull_filetered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->cloud_path.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->surface_hull.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void Trajectory::process() {
    // 1. 等待数据
    while (ros::ok()) {
        while (cloud_currnet->empty() && path_received.poses.empty()) {
            std::cout << "waiting for data" << std::endl;
            ros::Duration(1).sleep();
        }
        path_calculation(this->transformed_points);

        //发布路径
        sensor_msgs::PointCloud2 path_cloud;
        pcl::toROSMsg(*this->surface_hull, path_cloud);
        path_cloud.header.frame_id="lidar_link";
        path_cloud.header.stamp =ros::Time::now();
        pubPath.publish(path_cloud);
        //发布滤波点云
        sensor_msgs::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(*this->cloud_hull_filetered, point_cloud_msg);
        point_cloud_msg.header.frame_id="lidar_link";
        point_cloud_msg.header.stamp = ros::Time::now();
        pubCloud.publish(point_cloud_msg);

        //打印平均距离
        float x_average = 0.0f;
        for(const auto&data:cloud_hull_filetered->points){
            printf("x:%f\n",data.x);
            printf("y:%f\n",data.y);
            printf("z:%f\n",data.z);
            x_average+=data.x;
            x_average=x_average/cloud_hull_filetered->size();
        }
        printf("trajectory x distance: ---%f---\n",x_average);


        cloud_hull_filetered->clear();

        // view_point();
        ros::Duration(0.1).sleep();
    };
}

}  // namespace trajectory_nm