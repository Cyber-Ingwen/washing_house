#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "sensor_msgs/msg/point_cloud2.hpp"


class LidarOdometry
{
    public:
        float T[6];
        pcl::PointCloud<pcl::PointXYZI> pcn;
        pcl::PointCloud<pcl::PointXYZI> edge_points;
        pcl::PointCloud<pcl::PointXYZI> plane_points;
        pcl::PointCloud<pcl::PointXYZI> last_pcn;
        pcl::PointCloud<pcl::PointXYZI> last_edge_points;
        pcl::PointCloud<pcl::PointXYZI> last_plane_points;

        LidarOdometry();
        int feature_extraction(pcl::PointCloud<pcl::PointXYZI> cloud);
        int matching(void);
        pcl::PointCloud<pcl::PointXYZI> transform(pcl::PointCloud<pcl::PointXYZI> cloud);
};

