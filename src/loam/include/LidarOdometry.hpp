#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace Eigen;


class LidarOdometry
{
    public:
        pcl::PointCloud<pcl::PointXYZI> pcn;
        pcl::PointCloud<pcl::PointXYZI> edge_points;
        pcl::PointCloud<pcl::PointXYZI> plane_points;
        pcl::PointCloud<pcl::PointXYZI> last_pcn;
        pcl::PointCloud<pcl::PointXYZI> last_edge_points;
        pcl::PointCloud<pcl::PointXYZI> last_plane_points;
        MatrixXf J;
        VectorXf F;
        float T[6];
        int init_flag;

        LidarOdometry();
        int input(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
        int feature_extraction(pcl::PointCloud<pcl::PointXYZI> cloud);
        int NewtonGussian(void);

        int matching(float *T);
        pcl::PointCloud<pcl::PointXYZI> transform(pcl::PointCloud<pcl::PointXYZI> cloud, float *T);
        VectorXf _get_jacobi_edge(Vector3f p1, Vector3f p2, Vector3f p3, float *T);
        VectorXf _get_jacobi_plane(Vector3f p1, Vector3f p2, Vector3f p3, Vector3f p4, float *T);
};

