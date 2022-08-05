#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "sensor_msgs/msg/point_cloud2.hpp"


class LidarOdometry
{
    private:
        /* data */
    public:
        pcl::PointCloud<pcl::PointXYZI> pcn;

        LidarOdometry(/* args */)
        {

        }
        
        void feature_extraction(pcl::PointCloud<pcl::PointXYZI> cloud)
        {
            int len = cloud.points.size();
            for (int i = 0; i < len; i++)
            {
                if(i % 16 >= 4)
                {
                    pcn[i] = cloud[i];
                }
            }
        }
};

