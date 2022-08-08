#include "LidarOdometry.hpp"
#include <iostream>
#include "stdio.h"
#include <cmath>

using namespace Eigen;
using namespace std;

#include <ctime>

clock_t t0, t1;

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

int LidarOdometry::matching(float *T)
{
    auto raw_edge_points = edge_points;
    auto raw_edge_points_matrix = raw_edge_points.getMatrixXfMap(3, 8, 0);
    t0=clock();
    edge_points = this->transform(edge_points, T);
    t1=clock();
    
    
    auto edge_points_matrix = edge_points.getMatrixXfMap(3, 8, 0);
    auto last_edge_points_matrix = last_edge_points.getMatrixXfMap(3, 8, 0);

    cout << "len:" << edge_points.points.size() << endl;
    t0=clock();
    for (int i = 0; i < edge_points.points.size(); i++)
    {
        VectorXf edge_point = edge_points_matrix.col(i);
        VectorXf distance_vect(edge_points.points.size());

        for (int j = 0; j < last_edge_points.points.size(); j++)
        {
            VectorXf last_point = last_edge_points_matrix.col(j);
            float distance = (last_point - edge_point).norm();
            distance_vect(j) = distance;
        }

        ArrayXi index = ArrayXi::LinSpaced(distance_vect.size(), 0, distance_vect.size() - 1);
        auto rule = [distance_vect](float a, float b) -> bool{return distance_vect(a) < distance_vect(b);};
        //sort(index.data(), index.data() + index.size(), rule);
    }
    t1=clock();
    double endtime=(double)(t1-t0)/CLOCKS_PER_SEC;
    cout<<"Total time:"<<endtime*1000<<"ms"<<endl;

    return 1;
}

pcl::PointCloud<pcl::PointXYZI> LidarOdometry::transform(pcl::PointCloud<pcl::PointXYZI> cloud, float *T)
{
    auto pc_matrix = cloud.getMatrixXfMap(3, 8, 0);

    float alpha, beta, gamma;
    float delta_x, delta_y, delta_z;

    alpha = T[0];
    beta = T[1];
    gamma = T[2];
    delta_x = T[3];
    delta_y = T[4];
    delta_z = T[5];

    Matrix3f R;
    Vector3f t(delta_x, delta_y, delta_z);
    R <<cos(beta)*cos(gamma), -sin(gamma)*cos(beta), sin(beta),
        sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha), -sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma), -sin(alpha)*cos(beta),
        sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma), sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha), cos(alpha)*cos(beta);

    VectorXf v;
    for(int i = 0; i < pc_matrix.row(0).size(); i++)
    {
        v = pc_matrix.col(i);
        pc_matrix.col(i) = R * v.matrix() + t.matrix();
    }

    return cloud;
}
