#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/passthrough.h>
#include<pcl/filters/crop_box.h>

// 定义ROI区域
float max_x = 15.0;
float min_x = -3.0;
float max_y = 3.0;
float min_y = -3.0;
float max_z = 3;
float min_z = -1;

std::string lidar1_topic = "/mid360_front";
std::string lidar2_topic = "/mid360_back";

ros::Publisher sync_lidar1;
ros::Publisher sync_lidar2;
ros::Publisher lidar_fusion;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

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
    pass.setFilterLimits(min_x, max_x);
    pass.filter(pcl_roi);

    pass.setInputCloud(pcl_roi.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(pcl_roi);

    pass.setInputCloud(pcl_roi.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(pcl_roi);


    // 车身点云剔除
    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(pcl_roi.makeShared());
    crop.setMin(Eigen::Vector4f(-1.35, -0.75, -0.5, 1.0)); // Set the minimum point of the crop box
    crop.setMax(Eigen::Vector4f(1.35, 0.75, 0.5, 1.0));   // Set the maximum point of the crop box
    crop.setNegative(true); // Set to true to remove points inside the box
    crop.filter(pcl_roi);


    // 将合并后的点云数据发布到lidar_fusion话题
    sensor_msgs::PointCloud2 fusedCloud;
    pcl::toROSMsg(pcl_roi, fusedCloud);
    fusedCloud.header.frame_id = "mid360_frame";
    lidar_fusion.publish(fusedCloud);
    printf("Merged point clouds and published to lidar_fusion topic.\n");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "time_sync");
    ros::NodeHandle nh;

    printf("ros init ok!....\r\n");
    // 发布的话题
    sync_lidar1 = nh.advertise<sensor_msgs::PointCloud2>("/sync_topic_front", 100);
    sync_lidar2 = nh.advertise<sensor_msgs::PointCloud2>("/sync_topic_back", 100);
    lidar_fusion = nh.advertise<sensor_msgs::PointCloud2>("/lidar_fusion", 100);

    // 订阅的话题
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar1_sub(nh, lidar1_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar2_sub(nh, lidar2_topic, 1);

    // 时间同步
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), lidar1_sub, lidar2_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    ros::shutdown();

    return 0;
}
