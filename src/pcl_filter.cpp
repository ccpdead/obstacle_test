#include <pcl/point_types.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>

int main() {
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 填充点云数据
    // 这里只是一个示例，实际情况下你需要填充实际的点云数据
    cloud->width  = 5;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    // 创建凸包对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud);
    chull.reconstruct(*cloud_hull);

    // 创建 CropHull 过滤器并设置参数
    pcl::CropHull<pcl::PointXYZ> cropHullFilter;
    cropHullFilter.setDim(3);
    cropHullFilter.setInputCloud(cloud);
    std::vector<pcl::Vertices> hullIndices;
    pcl::toPCLPointCloud2(*chull.getIndices(), hullIndices);
    cropHullFilter.setHullIndices(hullIndices);
    cropHullFilter.setHullCloud(cloud_hull);

    // 应用过滤器
    cropHullFilter.filter(*cloud_filtered);

    return 0;
}