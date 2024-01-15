#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <sensor_msgs/PointCloud2.h>


class pcl_saved {
   public:
    pcl_saved(std::string name) {
        file_path = name;
        ros::NodeHandle nh;
        cloud_sub = nh.subscribe("/lidar_fusion", 1, &pcl_saved::cloudCallback, this);
        save_flag = false;
    }

   public:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        printf("received msg...\n");
        pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*msg, *cloud);
        pcl::io::savePCDFile("./"+file_path, *cloud);
    }

   private:
    ros::Subscriber cloud_sub;
    std::string file_path;
    pcl::PCDWriter writer;

   public:
    bool save_flag;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_saved");
    std::string path;
    path = argv[1];
    pcl_saved saved(argv[1]);
    printf("put s to saved pcd...\n");
    char input = 'a';
    std::cin >> input;
    if (input == 's') {
        saved.save_flag = true;
    }
    while (ros::ok())
    {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    static int i=0;
    if(i++>3)
      break;
      /* code */
    }
    
    return 0;
}