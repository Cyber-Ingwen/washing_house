#include "rclcpp/rclcpp.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "sensor_msgs/msg/point_cloud2.hpp"


class LidarOdometry
{
    public:
        pcl::PointCloud<pcl::PointXYZI> pcn;
        pcl::PointCloud<pcl::PointXYZI> edge_points;
        pcl::PointCloud<pcl::PointXYZI> plane_points;

        LidarOdometry(/* args */);
        
        pcl::PointCloud<pcl::PointXYZI> feature_extraction(pcl::PointCloud<pcl::PointXYZI> cloud);
};

