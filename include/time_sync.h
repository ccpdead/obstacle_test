#ifndef __TIME_SYNC_H__
#define __TIME_SYNC_H__

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

namespace TIME_SYNC_NS {

class Time_Sync {
   public:
    Time_Sync();
    ~Time_Sync();

   private:
   public:
    ros::Publisher sync_lidar1;
    ros::Publisher sync_lidar2;
    ros::Publisher lidar_fusion;
};
}  // namespace TIME_SYNC_NS
#endif