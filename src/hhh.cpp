#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
void test() {
    tf2_ros::Buffer tf_buffer;
   

    // 创建输入和输出的 PoseStamped 变量
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::PoseStamped pose_stamped_base_link;

    // 填充 pose_stamped 的数据
    pose_stamped.header.frame_id = "source_frame";  // 替换为实际的源坐标系
    pose_stamped.pose.position.x = 1.0;
    pose_stamped.pose.position.y = 2.0;
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation.w = 1.0;

    try {
        // 执行坐标转换，将 pose_stamped 从 "source_frame" 坐标系转换到 "base_link" 坐标系
        tf_buffer.transform(pose_stamped, pose_stamped_base_link, "base_link");
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Failed to transform pose: %s", ex.what());
        // return 1;
    }

    // 输出转换后的坐标
    ROS_INFO("Transformed pose: x=%.2f, y=%.2f, z=%.2f", pose_stamped_base_link.pose.position.x, pose_stamped_base_link.pose.position.y,
             pose_stamped_base_link.pose.position.z);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "tf2_example");
    ros::NodeHandle nh;
    test();
    // 创建 TransformListener
    test();
    return 0;
}
