#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

#include <visualization_msgs/Marker.h>

// 定义ROI区域
struct ROI {
    float max_x = 10.0;
    float min_x = -3.0;
    float max_y = 2.0;
    float min_y = -2.0;
    float max_z = 1.5;
    float min_z = -0.4;
} ROI;

struct car_box {
    float car_max_x = 1.35;
    float car_min_x = -1.35;
    float car_max_y = 0.75;
    float car_min_y = -0.75;
    float car_max_z = 0.5;
    float car_min_z = -0.5;
} car_box;

std::string lidar1_topic = "/mid360_front";
std::string lidar2_topic = "/mid360_back";

ros::Publisher sync_lidar1;
ros::Publisher sync_lidar2;
ros::Publisher lidar_fusion;
ros::Publisher marker_pub_car;
ros::Publisher marker_pub_arm;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

//marker_发布
void Marker_publisher() {
    visualization_msgs::Marker marker_car;
    visualization_msgs::Marker marker_arm;
    marker_car.header.frame_id = "mid360_frame";
    marker_car.header.stamp = ros::Time::now();
    marker_car.ns = "";
    marker_car.id = 0;
    marker_car.type = visualization_msgs::Marker::CUBE;
    marker_car.action = visualization_msgs::Marker::ADD;

    // 设置位置和姿态
    marker_car.pose.position.x = 0.0;
    marker_car.pose.position.y = 0.0;
    marker_car.pose.position.z = 0.0;
    marker_car.pose.orientation.x = 0.0;
    marker_car.pose.orientation.y = 0.0;
    marker_car.pose.orientation.z = 0.0;
    marker_car.pose.orientation.w = 1.0;

    // 设置大小
    marker_car.scale.x = car_box.car_max_x-car_box.car_min_x;
    marker_car.scale.y = car_box.car_max_y-car_box.car_min_y;
    marker_car.scale.z = car_box.car_max_z-car_box.car_min_z;

    // 设置颜色和透明度
    marker_car.color.r = 0.0;
    marker_car.color.g = 0.9;
    marker_car.color.b = 0.0;
    marker_car.color.a = 0.5;
    marker_car.lifetime = ros::Duration();  // 持续时间，0表示永久
    marker_pub_car.publish(marker_car);

    marker_arm = marker_car;
    marker_arm.pose.position.y = (marker_car.scale.y/2.0+1)*-1;

    marker_arm.scale.x = 0.5;
    marker_arm.scale.y = 2;
    marker_arm.scale.z = 1;

    marker_arm.color.r = 0.9;
    marker_arm.color.g = 0.0;
    marker_arm.color.b = 0.0;
    marker_pub_arm.publish(marker_arm);

}

// 回调函数
void callback(const sensor_msgs::PointCloud2ConstPtr& lidar1, const sensor_msgs::PointCloud2ConstPtr& lidar2) {
    // // 事实发布的话题
    // sync_lidar1.publish(*lidar1);
    // sync_lidar2.publish(*lidar2);
    // static int i;
    // printf("times:%d\r\n", i++);

    // 合并点云
    pcl::PointCloud<pcl::PointXYZI> pclCloud1;
    pcl::fromROSMsg(*lidar1, pclCloud1);

    pcl::PointCloud<pcl::PointXYZI> pclCloud2;
    pcl::fromROSMsg(*lidar2, pclCloud2);

    // 合并点云数据
    pcl::PointCloud<pcl::PointXYZI> mergedCloud;
    pcl::PointCloud<pcl::PointXYZI> pcl_roi;
    mergedCloud += pclCloud1;
    mergedCloud += pclCloud2;

    // ROI区域分割
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(mergedCloud.makeShared());
    pass.setFilterLimitsNegative(false);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(ROI.min_x, ROI.max_x);
    pass.filter(pcl_roi);

    pass.setInputCloud(pcl_roi.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(ROI.min_y, ROI.max_y);
    pass.filter(pcl_roi);

    pass.setInputCloud(pcl_roi.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(ROI.min_z, ROI.max_z);
    pass.filter(pcl_roi);

    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(pcl_roi.makeShared());
    crop.setMin(Eigen::Vector4f(car_box.car_min_x, car_box.car_min_y, car_box.car_min_z, 1.0));  // Set the minimum point of the crop box
    crop.setMax(Eigen::Vector4f(car_box.car_max_x, car_box.car_max_y, car_box.car_max_z, 1.0));  // Set the maximum point of the crop box
    crop.setNegative(true);                                                                      // Set to true to remove points inside the box
    crop.filter(pcl_roi);

    // 将合并后的点云数据发布到lidar_fusion话题
    sensor_msgs::PointCloud2 fusedCloud;
    pcl::toROSMsg(pcl_roi, fusedCloud);
    fusedCloud.header.frame_id = "mid360_frame";
    lidar_fusion.publish(fusedCloud);
    printf("fusedCloud size:%d\r\n", fusedCloud.data.size());

    //发布marker
    Marker_publisher();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "time_sync");
    ros::NodeHandle nh;
    printf("ros init ok!....\r\n");
    // 发布的话题
    sync_lidar1 = nh.advertise<sensor_msgs::PointCloud2>("/sync_topic_front", 100);
    sync_lidar2 = nh.advertise<sensor_msgs::PointCloud2>("/sync_topic_back", 100);
    lidar_fusion = nh.advertise<sensor_msgs::PointCloud2>("/lidar_fusion", 100);
    marker_pub_car = nh.advertise<visualization_msgs::Marker>("marker_car", 10);
    marker_pub_arm = nh.advertise<visualization_msgs::Marker>("marker_arm", 10);

    // 订阅的话题
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar1_sub(nh, lidar1_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar2_sub(nh, lidar2_topic, 1);

    // 时间同步
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), lidar1_sub, lidar2_sub);
    //boost::bind作用是将callback函数和参数_1,_2进行绑定
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    ros::shutdown();

    return 0;
}
