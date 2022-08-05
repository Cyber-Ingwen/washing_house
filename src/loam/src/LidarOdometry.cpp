#include "LidarOdometry.hpp"
#include <iostream>
#include "stdio.h"

LidarOdometry::LidarOdometry()
{

}

pcl::PointCloud<pcl::PointXYZI> LidarOdometry::feature_extraction(pcl::PointCloud<pcl::PointXYZI> cloud)
{
    /* 分割地面点 */
    pcl::PointXYZI point;
    for (int i = 0; i < cloud.points.size(); i++)
    {
        if(i % 16 >= 4)
        {
            pcn.points.push_back(cloud.points[i]);
        }
    }

    /* 提取竖线和平面 */
    for (int i = 0; i < pcn.points.size(); i++)
    {
        float x = pcn.points[i].x;
        float y = pcn.points[i].y;
        float z = pcn.points[i].z;

        float curv = 0;
        float sum[3] = {0, 0, 0};

        if((i - 12 * 5 >= 0 ) && (i + 12 * 5 < pcn.points.size()))
        {
            for (int j = 0; j < 5; j++)
            {
                int next_index = i + 12 * j;
                int last_index = i - 12 * j;
                sum[0] += (x - pcn.points[last_index].x);
                sum[0] += (x - pcn.points[next_index].x);
                sum[1] += (y - pcn.points[last_index].y);
                sum[1] += (y - pcn.points[next_index].y);
                sum[2] += (z - pcn.points[last_index].z);
                sum[2] += (z - pcn.points[next_index].z);
            }

            curv = sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2];
        }

        if((curv < 100) && (curv > 0.2))
        {
            edge_points.points.push_back(pcn.points[i]);
        }
        else if((curv < 2e-5) && (curv > 0))
        {
            plane_points.points.push_back(pcn.points[i]);
        }
    }

    return edge_points;
}
