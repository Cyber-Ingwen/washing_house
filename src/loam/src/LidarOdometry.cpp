#include "LidarOdometry.hpp"
#include <iostream>
#include "stdio.h"
#include <cmath>

using namespace Eigen;
using namespace std;

LidarOdometry::LidarOdometry()
{

}

int LidarOdometry::feature_extraction(pcl::PointCloud<pcl::PointXYZI> cloud)
{
    pcn.clear();
    edge_points.clear();
    plane_points.clear();
    last_pcn.clear();
    last_edge_points.clear();
    last_plane_points.clear();

    /* 分割地面点 */
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

    last_pcn = pcn;
    last_edge_points = edge_points;
    last_plane_points = plane_points;

    return 1;
}

int LidarOdometry::matching(void)
{
    pcl::PointCloud<pcl::PointXYZI> rew_edge_points = edge_points;
    edge_points = this->transform(edge_points);

    return 1;
}

pcl::PointCloud<pcl::PointXYZI> LidarOdometry::transform(pcl::PointCloud<pcl::PointXYZI> cloud)
{
    auto pc_matrix = cloud.getMatrixXfMap(3, 8, 0);
    std::cout << "p1:" << cloud.points[0] << std::endl;

    Matrix3f R;
    R <<2.0, 0.0, 0.0,
        0.0, 2.0, 0.0,
        0.0, 0.0, 2.0;
    
    VectorXf v;
    for(int i = 0; i < pc_matrix.row(0).size(); i++)
    {
        v = pc_matrix.col(i);
        pc_matrix.col(i) = R * v.matrix();
    }

    std::cout << "e:" << pc_matrix.col(0) << std::endl;
    std::cout << "p2:" << cloud.points[0] << std::endl;
    std::cout << "____________" << std::endl;

    return cloud;
}
