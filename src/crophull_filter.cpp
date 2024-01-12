#include <pcl/filters/crop_hull.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    if (argv[1] == NULL) {
        std::cout << "Please input the pcd file path!" << std::endl;
        return 0;
    }
    reader.read(argv[1], *cloud);

    float r = 5;
    float w = 0.5;

    // 定义2D平面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (float deg = M_PI; deg < 2 * M_PI; deg += 0.1) {
        // 内圆
        float x = cos(deg) * (r - w);
        float y = sin(deg) * (r - w) + r;
        float z = 0.0;

        boundingbox_ptr->push_back(pcl::PointXYZ(x, y, z));
        // }
        // 外圆
        float x2 = cos(deg) * (r + w);
        float y2 = sin(deg) * (r + w) + r;
        boundingbox_ptr->push_back(pcl::PointXYZ(x2, y2, z));
        // }
    }

    /**
     * 测试利用圆的分割来过滤点云
     */
    std::vector<int> oddNumbers;
    std::vector<int> evenNumbers;
    for (int i = 0; i < boundingbox_ptr->size(); ++i) {
        if (i % 2 == 0) {
            evenNumbers.push_back(i);
        } else {
            oddNumbers.push_back(i);
        }
    }
    // 对奇数和偶数进行从小到大排序

    auto comparator = [](int a, int b) { return a > b; };
    std::sort(oddNumbers.begin(), oddNumbers.end());
    std::sort(evenNumbers.begin(), evenNumbers.end(), comparator);

    std::vector<int> allNumbers;
    allNumbers.insert(allNumbers.end(), oddNumbers.begin(), oddNumbers.end());
    allNumbers.insert(allNumbers.end(), evenNumbers.begin(), evenNumbers.end());
    // 输出所有数
    for (const auto& num : allNumbers) {
        std::cout << num << " ";
    }
    std::cout << std::endl;

    // 输出奇数
    std::cout << "oddNumbers:" << std::endl;
    for (const auto& num : oddNumbers) {
        std::cout << num << " ";
    }
    std::cout << std::endl;

    // 输出偶数
    std::cout << "evenNumbers:" << std::endl;
    for (const auto& num : evenNumbers) {
        std::cout << num << " ";
    }
    std::cout << std::endl;

    /*-------------------------------------------------------------------------------------------------*/
    // pcl::ConvexHull<pcl::PointXYZ> hull;  // 创建凸包对象
    // // pcl::ConcaveHull<pcl::PointXYZ> hull;//创建凹包对象
    // hull.setInputCloud(boundingbox_ptr);                                                   // 设置输入点云
    // hull.setDimension(2);                                                                  // 设置凸包维度
    std::vector<pcl::Vertices> polygons;                                                   // 设置向量，用于保存凸包定点
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);  // 该点运用于描述凸包形状
    // hull.reconstruct(*surface_hull, polygons);                                             // 计算2D凸包结果

    // 添加凸包的顶点
    polygons.clear();

    pcl::Vertices vertics;
    for(const auto&num:allNumbers){
        vertics.vertices.push_back(num);
    }
    polygons.push_back(vertics);

    surface_hull->clear();
    surface_hull = boundingbox_ptr;
    // 打印数据
    for (const auto& element : polygons) {
        std::cout << element << " ";
    }
    std::cout << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropHull<pcl::PointXYZ> bb_filter;     // 创建crophull对象
    bb_filter.setDim(2);                        // 设置维度：该维度需要与凸包维度一致
    bb_filter.setInputCloud(cloud);             // 设置需要滤波的点云
    bb_filter.setHullIndices(polygons);         // 输入封闭多边形的顶点
    bb_filter.setHullCloud(surface_hull);       // 输入封闭多边形的形状
    bb_filter.setNegative(false);               // 设置为true表示保留凸包外的点，false表示保留凸包内的点
    bb_filter.filter(*objects);                 // 执行CropHull滤波，存出结果在objects
    std::cout << objects->size() << std::endl;  //

    // for (int i = 0; i < boundingbox_ptr->size(); ++i) {
    //     std::cout << boundingbox_ptr->points[i].x << " " << boundingbox_ptr->points[i].y << " " << boundingbox_ptr->points[i].z << std::endl;
    // }

    /************************************************************************************************************/
    // visualize
    boost::shared_ptr<pcl::visualization::PCLVisualizer> for_visualizer_v(new pcl::visualization::PCLVisualizer("crophull display"));
    for_visualizer_v->setBackgroundColor(255, 255, 255);

    int v1(0);  // 显示原始点云
    for_visualizer_v->createViewPort(0.0, 0.0, 0.33, 1, v1);
    for_visualizer_v->setBackgroundColor(255, 255, 255, v1);
    for_visualizer_v->addPointCloud(cloud, "cloud", v1);
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "cloud");
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    for_visualizer_v->addPolygon<pcl::PointXYZ>(boundingbox_ptr, 0, .069 * 255, 0.2 * 255, "backview_hull_polyline1", v1);

    int v2(0);  // 显示封闭2D多边形凸包
    for_visualizer_v->createViewPort(0.33, 0.0, 0.66, 1, v2);
    for_visualizer_v->setBackgroundColor(255, 255, 255, v2);
    for_visualizer_v->addPointCloud(surface_hull, "surface_hull", v2);
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "surface_hull");
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "surface_hull");
    for_visualizer_v->addPolygon<pcl::PointXYZ>(surface_hull, 0, .069 * 255, 0.2 * 255, "backview_hull_polyline", v2);
    // addPolygon函数：添加表示输入点云的多边形（折线、全部连接）

    int v3(0);  // 显示滤波结果
    for_visualizer_v->createViewPort(0.66, 0.0, 1, 1, v3);
    for_visualizer_v->setBackgroundColor(255, 255, 255, v3);
    for_visualizer_v->addPointCloud(objects, "objects", v3);
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "objects");
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "objects");

    // 添加pcl坐标轴显示
    for_visualizer_v->addCoordinateSystem(1.0);
    for_visualizer_v->initCameraParameters();
    while (!for_visualizer_v->wasStopped()) {
        for_visualizer_v->spinOnce(1000);
    }
    system("pause");
    return 0;
}