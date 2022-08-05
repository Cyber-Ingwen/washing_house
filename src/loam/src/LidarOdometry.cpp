#include "LidarOdometry.hpp"


LidarOdometry::LidarOdometry(/* args */)
{

}

void LidarOdometry::feature_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr)
{
    int len = cloud_ptr->points.size();
    for (int i = 0; i < len; i++)
    {
        if(i % 16 >= 4)
        {
            pcn[i] = cloud_ptr->points[i];
        }
    }
}