#include "obscalte_test.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "trajectory_leapting");
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer;                         // 使用tf进行坐标转换
    tf2_ros::TransformListener tfListener(tfBuffer);  // 启动监听
    tfBuffer.setUsingDedicatedThread(true);
    std::unique_ptr<trajectory_nm::Trajectory> trajectory_(new trajectory_nm::Trajectory(nh_,tfBuffer));
    constexpr size_t num_threads = 0;
    if(num_threads>1){
      ros::AsyncSpinner spinner(num_threads);
      spinner.start();
      ros::waitForShutdown();
    }else{
      ros::spin();
    }
    return 0;
}