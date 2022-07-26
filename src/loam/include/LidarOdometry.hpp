#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>

using namespace Eigen;
using namespace std;


class LidarOdometry
{
    public:
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcn;
        pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points;
        pcl::PointCloud<pcl::PointXYZI>::Ptr plane_points;
        pcl::PointCloud<pcl::PointXYZI>::Ptr last_pcn;
        pcl::PointCloud<pcl::PointXYZI>::Ptr last_edge_points;
        pcl::PointCloud<pcl::PointXYZI>::Ptr last_plane_points;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtreeCornerFromMap;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtreeSurfFromMap;

        MatrixXf J;
        VectorXf F;
        float T[6];
        int init_flag;

        LidarOdometry();
        int input(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
        int feature_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
        int NewtonGussian(void);
        int LevenbergMarquardt(void);
        int matching(float *T);
        void kdMatching(float *T);

        pcl::PointCloud<pcl::PointXYZI>::Ptr transform(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float *T);
        VectorXf _get_jacobi_edge(Vector3f p1, Vector3f p2, Vector3f p3, float *T);
        VectorXf _get_jacobi_plane(Vector3f p1, Vector3f p2, Vector3f p3, Vector3f p4, float *T);

        pcl::PointCloud<pcl::PointXYZI> test_point_1;
        pcl::PointCloud<pcl::PointXYZI> test_point_2;
        int test_flag;

        vector<float *> T_list;
        vector<int> pointSearchInd;
        vector<float> pointSearchSqDis;
};

