#ifndef _PATH_DISPLAY_H_
#define _PATH_DISPLAY_H_

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace pathdisplay {
class PathDisplay {
   public:
    explicit PathDisplay(ros::NodeHandle nh);  // 显式构造函数
    ~PathDisplay();

   private:
    ros::Subscriber subPath;  // 订阅路径
    ros::Publisher pubPath;   // 发布路径

   private:
    void path_Callback(const nav_msgs::Path::ConstPtr& msg);  // 回调函数
};

}  // namespace pathdisplay

#endif  //_PATH_DISPLAY_H_