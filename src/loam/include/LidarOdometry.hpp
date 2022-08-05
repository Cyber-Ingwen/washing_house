#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "sensor_msgs/msg/point_cloud2.hpp"


class LidarOdometry
{
    public:
        pcl::PointCloud<pcl::PointXYZI> pcn;

        LidarOdometry(/* args */);
        
        void feature_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
};

