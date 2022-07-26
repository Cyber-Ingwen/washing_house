#include "LidarOdometry.hpp"
#include <iostream>
#include "stdio.h"
#include <cmath>
#include <random>

using namespace Eigen;
using namespace std;


LidarOdometry::LidarOdometry()
{
    init_flag = 0;
    last_edge_points = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    last_plane_points = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    last_pcn = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    edge_points = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    plane_points = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcn = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    // kdtreeCornerFromMap = boost::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
    // kdtreeSurfFromMap = boost::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
}

int LidarOdometry::input(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr)
{
    if (init_flag == 0)
    {
        this->feature_extraction(cloud_ptr);
        *last_edge_points = *edge_points;
        *last_plane_points = *plane_points;
        *last_pcn = *pcn;

        init_flag = 1;
    }
    else if (init_flag == 1)
    {
        this->feature_extraction(cloud_ptr);
        this->LevenbergMarquardt();

        T_list.push_back(T);

        *last_edge_points = *edge_points;
        *last_plane_points = *plane_points;
        *last_pcn = *pcn;
    }

    return 1;
}

int LidarOdometry::feature_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcn->clear();
    edge_points->clear();
    plane_points->clear();

    /* 分割地面点 */
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if(i % 16 >= 4)
        {
            pcn->points.push_back(cloud->points[i]);
        }
    }
    /* 提取竖线和平面 */
    for (int sector = 0; sector < 6; sector++)
    {
        vector<float> curv_list;
        for (int i = int(pcn->points.size() / 6) * sector; i < int(pcn->points.size() / 6) * (sector + 1); i++)
        {
            float x = pcn->points[i].x;
            float y = pcn->points[i].y;
            float z = pcn->points[i].z;
            float r0 = x*x + y*y + z*z;

            float curv = 0;
            float sum[3] = {0, 0, 0};
            float sum2 = 0;

            if((i - 12 * 5 >= 0 ) && (i + 12 * 5 < pcn->points.size()))
            {
                for (int j = 0; j < 5; j++)
                {
                    int next_index = i + 12 * j;
                    int last_index = i - 12 * j;
                    sum[0] += (x - pcn->points[last_index].x);
                    sum[0] += (x - pcn->points[next_index].x);
                    sum[1] += (y - pcn->points[last_index].y);
                    sum[1] += (y - pcn->points[next_index].y);
                    sum[2] += (z - pcn->points[last_index].z);
                    sum[2] += (z - pcn->points[next_index].z);

                    sum2 += (x - pcn->points[last_index].x) * (x - pcn->points[last_index].x) + (y - pcn->points[last_index].y) * (y - pcn->points[last_index].y) + (z - pcn->points[last_index].z) * (z - pcn->points[last_index].z);
                    sum2 += (x - pcn->points[next_index].x) * (x - pcn->points[next_index].x) + (y - pcn->points[next_index].y) * (y - pcn->points[next_index].y) + (z - pcn->points[next_index].z) * (z - pcn->points[next_index].z);
                }

                curv = (sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]) / (sum2);
                if (isnan(curv)){ curv = -1;}
                if (r0 < 10){ curv = -1;}

                int next_index = i + 12 * 5;
                int last_index = i - 12 * 5;
                float rl = pcn->points[last_index].x * pcn->points[last_index].x + pcn->points[last_index].y * pcn->points[last_index].y + pcn->points[last_index].z * pcn->points[last_index].z;
                float rn = pcn->points[next_index].x * pcn->points[next_index].x + pcn->points[next_index].y * pcn->points[next_index].y + pcn->points[next_index].z * pcn->points[next_index].z;
                if ((abs(rl - r0) / r0 > 0.2) || (abs(rn - r0) / r0 > 0.2))
                {
                    curv = -1;
                }

                curv_list.push_back(curv);
            }
        }
        
        vector<int> index(curv_list.size());
        for (int ind = 0; ind < curv_list.size(); ind++) {index[ind] = ind;}
        auto rule = [curv_list](int a, int b) -> bool{return curv_list[a] < curv_list[b];};
        sort(index.begin(), index.end(), rule);

        vector<int> plane_index; 
        vector<int> edge_index;

        for (int j = 0; j < index.size(); j++)
        {
            if (plane_index.size() >= 48)
            {
                break;
            }
            if (curv_list[index[j]] > 0)
            {
                int flag = 1;
                for (int k = 0; k < 5; k++)
                {
                    if (index[j] + k < curv_list.size())
                    {
                        if (curv_list[index[j]] > curv_list[index[j] + k]) {flag = 0;}
                    }
                    if (index[j] - k > 0)
                    {
                        if (curv_list[index[j]] > curv_list[index[j] - k]) {flag = 0;}
                    }
                }
                if (flag == 1)
                {
                    plane_index.push_back(index[j]);
                }
            }
        }

        reverse(index.begin(), index.end());
        for (int j = 0; j < index.size(); j++)
        {
            if (edge_index.size() >= 24)
            {
                break;
            }
            if ((curv_list[index[j]] < 100) && (curv_list[index[j]] > 0))
            {
                int flag = 1;
                for (int k = 0; k < 5; k++)
                {
                    if (index[j] + k < curv_list.size())
                    {
                        if (curv_list[index[j]] < curv_list[index[j] + k]) {flag = 0;}
                    }
                    if (index[j] - k > 0)
                    {
                        if (curv_list[index[j]] < curv_list[index[j] - k]) {flag = 0;}
                    }
                }
                if (flag == 1)
                {
                    edge_index.push_back(index[j]);
                }
            }
        }

        for (int ind : plane_index)
        {
            ind += int(pcn->points.size() / 6) * sector;
            if (sector == 0)
            {
                ind += 12 * 5;
            }
            pcn->points[ind].intensity = ind;
            plane_points->points.push_back(pcn->points[ind]);
        }

        for (int ind : edge_index)
        {
            ind += int(pcn->points.size() / 6) * sector;
            if (sector == 0)
            {
                ind += 12 * 5;
            }
            pcn->points[ind].intensity = ind;
            edge_points->points.push_back(pcn->points[ind]);
        }

        // kdtreeCornerFromMap->setInputCloud(edge_points);
		// kdtreeSurfFromMap->setInputCloud(plane_points);
    }

    return 1;
}

int LidarOdometry::NewtonGussian(void)
{
    /* 牛顿高斯法优化 */
    float x[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    cout << "__________" << endl;
    VectorXf x_vect(6);
    for (int num = 0; num < 10; num++)
    {
        this->matching(x);

        MatrixXf temp = ((J * J.transpose()).inverse()) * J * F.matrix();
        x_vect = VectorXf::Zero(6);
        x_vect << x[0], x[1], x[2], x[3], x[4], x[5];
        x_vect = x_vect.matrix() - 4 * temp;
        x[0] = x_vect(0);
        x[1] = x_vect(1);
        x[2] = x_vect(2);
        x[3] = x_vect(3);
        x[4] = x_vect(4);
        x[5] = x_vect(5);

        memcpy(T, x, sizeof(x));

        cout << "\033c" << flush;
        cout << "x: " << x_vect.norm() << " f: " << F.norm() << "delta: " << temp.transpose() << endl;
        if (F.norm() < 15) {break;}
    }

    return 1;
}

int LidarOdometry::LevenbergMarquardt(void)
{
    /* 列文伯格-马夸尔特算法优化 */
    float x[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float x_new[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    this->matching(x);

    double t = 1e-5;
    double v = 2.0;
    MatrixXf g = J * F.matrix();
    MatrixXf H = J * J.transpose();
    double u = t * H.maxCoeff();
    auto F_last = F.norm();
    cout << "__________" << endl;

    for (int num = 0; num < 16; num++)  
    {
        VectorXf h = (H + u * MatrixXf::Identity(6, 6)).inverse() * g;
        VectorXf x_vect = VectorXf::Zero(6);

        x_vect << x[0], x[1], x[2], x[3], x[4], x[5];
        x_vect = x_vect - h;
        x_new[0] = x_vect(0);
        x_new[1] = x_vect(1);
        x_new[2] = x_vect(2);
        x_new[3] = x_vect(3);
        x_new[4] = x_vect(4);
        x_new[5] = x_vect(5);
        this->matching(x_new);

        auto p = (F_last - F.norm()) / double(0.5 * h.matrix().transpose() * (u * h - g));
        if (p > 0)
        {
            memcpy(x, x_new, sizeof(x_new));
            H = J * J.transpose();
            g = J * F.matrix();
            F_last = F.norm();
            u = u * max(0.3, (1 - pow((2 * p - 1), 3)));
            v = 2.0;
        }
        else 
        {
            cout<<"else"<<endl;
            u = u * v;
            v = 2.0 * v;
        }

        memcpy(T, x, sizeof(x));

        cout << "\033c" << flush;
        cout << "x: " << x_vect.norm() << " f: " << F.norm() << endl;
        if (F.norm() < 15) {break;}
    }

    return 1;
}


// void LidarOdometry::kdMatching(float *T)
// {
//     J = MatrixXf::Zero(6, edge_points->points.size() + plane_points->points.size());
//     F = VectorXf::Zero(edge_points->points.size() + plane_points->points.size());

//     auto raw_edge_points = edge_points;
//     edge_points = this->transform(edge_points, T);

//     for (int i = 0; i < edge_points->points.size(); i++)
//     {
//         kdtreeCornerLast->nearestKSearch(edge_points->points, 2, pointSearchInd, pointSearchSqDis);

//         int nearest_index, near_angle_index;
//         nearest_index = pointSearchInd[0];
//         near_angle_index = pointSearchInd[1];

//     }

//     return;
// }

int LidarOdometry::matching(float *T)
{
    /* 特征点匹配 */
    J = MatrixXf::Zero(6, edge_points->points.size() + plane_points->points.size());
    F = VectorXf::Zero(edge_points->points.size() + plane_points->points.size());
    auto last_pcn_matrix = last_pcn->getMatrixXfMap(3, 8, 0);
    
    /* 边缘点匹配 */
    auto raw_edge_points = edge_points;
    auto raw_edge_points_matrix = raw_edge_points->getMatrixXfMap(3, 8, 0);

    edge_points = this->transform(edge_points, T);
    auto edge_points_matrix = edge_points->getMatrixXfMap(3, 8, 0);
    auto last_edge_points_matrix = last_edge_points->getMatrixXfMap(3, 8, 0);
    
    for (int i = 0; i < edge_points->points.size(); i++)
    {
        Vector3f edge_point = edge_points_matrix.col(i);
        VectorXf distance_vect(last_edge_points->points.size());

        for (int j = 0; j < last_edge_points->points.size(); j++)
        {
            Vector3f last_point = last_edge_points_matrix.col(j);
            float distance = (last_point - edge_point).norm();
            distance_vect(j) = distance;
        }

        ArrayXi index = ArrayXi::LinSpaced(distance_vect.size(), 0, distance_vect.size() - 1);
        auto rule = [distance_vect](float a, float b) -> bool{return distance_vect(a) < distance_vect(b);};
        partial_sort(index.data(), index.data() + 2, index.data() + index.size(), rule);

        int nearest_index, near_angle_index;
        nearest_index = index(0);
        near_angle_index = index(1);
        
        Vector3f p1 = raw_edge_points_matrix.col(i);
        Vector3f p2 = last_edge_points_matrix.col(nearest_index);
        Vector3f p3 = last_edge_points_matrix.col(near_angle_index);

        float d = (p2 - p3).norm();
        float s = ((p2 - edge_point).cross(p3 - edge_point)).norm();
        float h = (s / d);

        J.col(i) = this->_get_jacobi_edge(p1, p2, p3, T);
        F(i) = h;

        //if (isnan(h)){cout<<"WARNING H"<<endl;}
        //if (isnan((J.col(i))(0))){cout<<"WARNING J"<<endl;}
        if (d == 0.0) {cout<<"WARNING s"<<endl;}

        Vector3f test = (p1 - p3).cross(p1 - p2);
        if (test.norm() == 0.0) {cout<<"WARNING 共线"<<endl;}

        if (test_flag == 0)
        {
            test_point_1.push_back(raw_edge_points->points[i]);
            test_point_2.push_back(last_edge_points->points[nearest_index]);
            test_point_2.push_back(last_edge_points->points[near_angle_index]);
            test_flag = 1;
        }
    }

    /* 平面点匹配 */
    auto raw_plane_points = plane_points;
    auto raw_plane_points_matrix = raw_plane_points->getMatrixXfMap(3, 8, 0);

    plane_points = this->transform(plane_points, T);
    auto plane_points_matrix = plane_points->getMatrixXfMap(3, 8, 0);
    auto last_plane_points_matrix = last_plane_points->getMatrixXfMap(3, 8, 0);

    for (int i = 0; i < plane_points->points.size(); i++)
    {
        Vector3f plane_point = plane_points_matrix.col(i);
        VectorXf distance_vect(last_plane_points->points.size());

        for (int j = 0; j < last_plane_points->points.size(); j++)
        {
            Vector3f last_point = last_plane_points_matrix.col(j);
            float distance = (last_point - plane_point).norm();
            distance_vect(j) = distance;
        }

        ArrayXi index = ArrayXi::LinSpaced(distance_vect.size(), 0, distance_vect.size() - 1);
        auto rule = [distance_vect](float a, float b) -> bool{return distance_vect(a) < distance_vect(b);};
        partial_sort(index.data(), index.data() + 3, index.data() + index.size(), rule);

        int nearest_index, near_angle_index, near_scan_index;
        nearest_index = index(0);
        near_angle_index = index(1);
        near_scan_index = index(2);
        
        Vector3f p1 = raw_plane_points_matrix.col(i);
        Vector3f p2 = last_plane_points_matrix.col(nearest_index);
        Vector3f p3 = last_plane_points_matrix.col(near_angle_index);
        Vector3f p4 = last_plane_points_matrix.col(near_scan_index);

        Vector3f s = (p2 - p3).cross(p2 - p4);
        float h = (p2 - plane_point).dot(s / s.norm());

        J.col(edge_points->points.size() + i) = this->_get_jacobi_plane(p1, p2, p3, p4, T);
        F(edge_points->points.size() + i) = h;

        //if (s.norm() == 0.0) {cout<<"WARNING 共线"<<endl;}
        //if (isnan(s.norm())){cout<<"WARNING PL S"<<endl;}
        if (isnan(h))
        {
            cout<<"WARNING PL H"<<endl;
            while (1)
            {
                /* code */
            }
            
            h = 0;
            VectorXf j = VectorXf::Zero(6);
            j << 0, 0, 0, 0, 0, 0;
            J.col(edge_points->points.size() + i) = j;
        }
        if (isnan((J.col(i))(0)))
        {
            cout<<"WARNING PL J"<<endl;
            VectorXf j = VectorXf::Zero(6);
            j << 0, 0, 0, 0, 0, 0;
            J.col(edge_points->points.size() + i) = j;
        }
    }

    edge_points = raw_edge_points;
    plane_points = raw_plane_points;

    return 1;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarOdometry::transform(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float *T)
{
    auto pc_matrix = cloud->getMatrixXfMap(3, 8, 0);
    float alpha, beta, gamma, delta_x, delta_y, delta_z;

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

VectorXf LidarOdometry::_get_jacobi_edge(Vector3f p1, Vector3f p2, Vector3f p3, float *T)
{
    float alpha, beta, gamma, x, y, z;
    float x_1, y_1, z_1, x_2, y_2, z_2, x_3, y_3, z_3;

    alpha = T[0];
    beta = T[1];
    gamma = T[2];
    x = T[3];
    y = T[4];
    z = T[5];

    x_1 = p1(0);
    y_1 = p1(1);
    z_1 = p1(2);
    x_2 = p2(0);
    y_2 = p2(1);
    z_2 = p2(2);
    x_3 = p3(0);
    y_3 = p3(1);
    z_3 = p3(2);

    float j11 = (2*(x_1*(-sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)) + y_1*(-sin(alpha)*cos(gamma) - sin(beta)*sin(gamma)*cos(alpha)) - z_1*cos(alpha)*cos(beta))*(-x - x_1*cos(beta)*cos(gamma) + x_2 + y_1*sin(gamma)*cos(beta) - z_1*sin(beta)) + 2*(x_1*(-sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)) + y_1*(-sin(alpha)*cos(gamma) - sin(beta)*sin(gamma)*cos(alpha)) - z_1*cos(alpha)*cos(beta))*(x + x_1*cos(beta)*cos(gamma) - x_3 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta)))*(-(x + x_1*cos(beta)*cos(gamma) - x_2 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)) + (x + x_1*cos(beta)*cos(gamma) - x_3 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_2 - z_1*sin(alpha)*cos(beta))) + (2*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - z_1*sin(alpha)*cos(beta))*(-x - x_1*cos(beta)*cos(gamma) + x_3 + y_1*sin(gamma)*cos(beta) - z_1*sin(beta)) + 2*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - z_1*sin(alpha)*cos(beta))*(x + x_1*cos(beta)*cos(gamma) - x_2 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta)))*((x + x_1*cos(beta)*cos(gamma) - x_2 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3) - (x + x_1*cos(beta)*cos(gamma) - x_3 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2)) + ((x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2)*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)) - (x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3)*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_2 - z_1*sin(alpha)*cos(beta)))*(2*(-x_1*(-sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)) - y_1*(-sin(alpha)*cos(gamma) - sin(beta)*sin(gamma)*cos(alpha)) + z_1*cos(alpha)*cos(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3) + 2*(x_1*(-sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)) + y_1*(-sin(alpha)*cos(gamma) - sin(beta)*sin(gamma)*cos(alpha)) - z_1*cos(alpha)*cos(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2) + 2*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - z_1*sin(alpha)*cos(beta))*(-x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) - y - y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) + y_2 + z_1*sin(alpha)*cos(beta)) + 2*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - z_1*sin(alpha)*cos(beta))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)));
    float j12 = ((x + x_1*cos(beta)*cos(gamma) - x_2 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3) - (x + x_1*cos(beta)*cos(gamma) - x_3 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2))*(2*(-x_1*sin(beta)*cos(gamma) + y_1*sin(beta)*sin(gamma) + z_1*cos(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3) + 2*(x_1*sin(beta)*cos(gamma) - y_1*sin(beta)*sin(gamma) - z_1*cos(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2) + 2*(-x_1*cos(alpha)*cos(beta)*cos(gamma) + y_1*sin(gamma)*cos(alpha)*cos(beta) - z_1*sin(beta)*cos(alpha))*(-x - x_1*cos(beta)*cos(gamma) + x_3 + y_1*sin(gamma)*cos(beta) - z_1*sin(beta)) + 2*(-x_1*cos(alpha)*cos(beta)*cos(gamma) + y_1*sin(gamma)*cos(alpha)*cos(beta) - z_1*sin(beta)*cos(alpha))*(x + x_1*cos(beta)*cos(gamma) - x_2 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))) + (-(x + x_1*cos(beta)*cos(gamma) - x_2 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)) + (x + x_1*cos(beta)*cos(gamma) - x_3 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_2 - z_1*sin(alpha)*cos(beta)))*(2*(-x_1*sin(beta)*cos(gamma) + y_1*sin(beta)*sin(gamma) + z_1*cos(beta))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_2 - z_1*sin(alpha)*cos(beta)) + 2*(x_1*sin(beta)*cos(gamma) - y_1*sin(beta)*sin(gamma) - z_1*cos(beta))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)) + 2*(x_1*sin(alpha)*cos(beta)*cos(gamma) - y_1*sin(alpha)*sin(gamma)*cos(beta) + z_1*sin(alpha)*sin(beta))*(-x - x_1*cos(beta)*cos(gamma) + x_2 + y_1*sin(gamma)*cos(beta) - z_1*sin(beta)) + 2*(x_1*sin(alpha)*cos(beta)*cos(gamma) - y_1*sin(alpha)*sin(gamma)*cos(beta) + z_1*sin(alpha)*sin(beta))*(x + x_1*cos(beta)*cos(gamma) - x_3 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))) + ((x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2)*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)) - (x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3)*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_2 - z_1*sin(alpha)*cos(beta)))*(2*(-x_1*sin(alpha)*cos(beta)*cos(gamma) + y_1*sin(alpha)*sin(gamma)*cos(beta) - z_1*sin(alpha)*sin(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3) + 2*(x_1*sin(alpha)*cos(beta)*cos(gamma) - y_1*sin(alpha)*sin(gamma)*cos(beta) + z_1*sin(alpha)*sin(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2) + 2*(-x_1*cos(alpha)*cos(beta)*cos(gamma) + y_1*sin(gamma)*cos(alpha)*cos(beta) - z_1*sin(beta)*cos(alpha))*(-x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) - y - y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) + y_2 + z_1*sin(alpha)*cos(beta)) + 2*(-x_1*cos(alpha)*cos(beta)*cos(gamma) + y_1*sin(gamma)*cos(alpha)*cos(beta) - z_1*sin(beta)*cos(alpha))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)));
    float j13 = ((x + x_1*cos(beta)*cos(gamma) - x_2 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3) - (x + x_1*cos(beta)*cos(gamma) - x_3 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2))*(2*(x_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + y_1*(-sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)))*(-x - x_1*cos(beta)*cos(gamma) + x_3 + y_1*sin(gamma)*cos(beta) - z_1*sin(beta)) + 2*(x_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + y_1*(-sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)))*(x + x_1*cos(beta)*cos(gamma) - x_2 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta)) + 2*(-x_1*sin(gamma)*cos(beta) - y_1*cos(beta)*cos(gamma))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3) + 2*(x_1*sin(gamma)*cos(beta) + y_1*cos(beta)*cos(gamma))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2)) + (-(x + x_1*cos(beta)*cos(gamma) - x_2 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)) + (x + x_1*cos(beta)*cos(gamma) - x_3 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_2 - z_1*sin(alpha)*cos(beta)))*(2*(x_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) + y_1*(-sin(alpha)*sin(beta)*cos(gamma) - sin(gamma)*cos(alpha)))*(-x - x_1*cos(beta)*cos(gamma) + x_2 + y_1*sin(gamma)*cos(beta) - z_1*sin(beta)) + 2*(x_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) + y_1*(-sin(alpha)*sin(beta)*cos(gamma) - sin(gamma)*cos(alpha)))*(x + x_1*cos(beta)*cos(gamma) - x_3 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta)) + 2*(-x_1*sin(gamma)*cos(beta) - y_1*cos(beta)*cos(gamma))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_2 - z_1*sin(alpha)*cos(beta)) + 2*(x_1*sin(gamma)*cos(beta) + y_1*cos(beta)*cos(gamma))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta))) + ((x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2)*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)) - (x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3)*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_2 - z_1*sin(alpha)*cos(beta)))*(2*(x_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + y_1*(-sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)))*(-x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) - y - y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) + y_2 + z_1*sin(alpha)*cos(beta)) + 2*(x_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + y_1*(-sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)) + 2*(-x_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_1*(-sin(alpha)*sin(beta)*cos(gamma) - sin(gamma)*cos(alpha)))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3) + 2*(x_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) + y_1*(-sin(alpha)*sin(beta)*cos(gamma) - sin(gamma)*cos(alpha)))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2));
    float j14 = (-2*y_2 + 2*y_3)*(-(x + x_1*cos(beta)*cos(gamma) - x_2 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)) + (x + x_1*cos(beta)*cos(gamma) - x_3 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_2 - z_1*sin(alpha)*cos(beta))) + (2*z_2 - 2*z_3)*((x + x_1*cos(beta)*cos(gamma) - x_2 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3) - (x + x_1*cos(beta)*cos(gamma) - x_3 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2));
    float j15 = (2*x_2 - 2*x_3)*(-(x + x_1*cos(beta)*cos(gamma) - x_2 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)) + (x + x_1*cos(beta)*cos(gamma) - x_3 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_2 - z_1*sin(alpha)*cos(beta))) + (-2*z_2 + 2*z_3)*((x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2)*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)) - (x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3)*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_2 - z_1*sin(alpha)*cos(beta)));
    float j16 = (-2*x_2 + 2*x_3)*((x + x_1*cos(beta)*cos(gamma) - x_2 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3) - (x + x_1*cos(beta)*cos(gamma) - x_3 - y_1*sin(gamma)*cos(beta) + z_1*sin(beta))*(x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2)) + (2*y_2 - 2*y_3)*((x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_2)*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_3 - z_1*sin(alpha)*cos(beta)) - (x_1*(sin(alpha)*sin(gamma) - sin(beta)*cos(alpha)*cos(gamma)) + y_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + z + z_1*cos(alpha)*cos(beta) - z_3)*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - y_2 - z_1*sin(alpha)*cos(beta)));

    VectorXf j(6);
    j = VectorXf::Zero(6);
    j << j11, j12, j13, j14, j15, j16;
    j /= ((-x_2 + x_3)*(-x_2 + x_3) + (-y_2 + y_3)*(-y_2 + y_3) + (-z_2 + z_3)*(-z_2 + z_3));

    return j;
}

VectorXf LidarOdometry::_get_jacobi_plane(Vector3f p1, Vector3f p2, Vector3f p3, Vector3f p4, float *T)
{
    float alpha, beta, gamma, x, y, z;
    float x_1, y_1, z_1, x_2, y_2, z_2, x_3, y_3, z_3, x_4, y_4, z_4;

    alpha = T[0];
    beta = T[1];
    gamma = T[2];
    x = T[3];
    y = T[4];
    z = T[5];

    x_1 = p1(0);
    y_1 = p1(1);
    z_1 = p1(2);
    x_2 = p2(0);
    y_2 = p2(1);
    z_2 = p2(2);
    x_3 = p3(0);
    y_3 = p3(1);
    z_3 = p3(2);
    x_4 = p4(0);
    y_4 = p4(1);
    z_4 = p4(2);

    float j21 = ((-x_2 + x_3)*(y_3 - y_4) - (x_3 - x_4)*(-y_2 + y_3))*(x_1*(sin(alpha)*sin(beta)*cos(gamma) + sin(gamma)*cos(alpha)) + y_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) - z_1*sin(alpha)*cos(beta)) + (-(-x_2 + x_3)*(z_3 - z_4) + (x_3 - x_4)*(-z_2 + z_3))*(x_1*(-sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)) + y_1*(-sin(alpha)*cos(gamma) - sin(beta)*sin(gamma)*cos(alpha)) - z_1*cos(alpha)*cos(beta));
    float j22 = ((-x_2 + x_3)*(y_3 - y_4) - (x_3 - x_4)*(-y_2 + y_3))*(-x_1*cos(alpha)*cos(beta)*cos(gamma) + y_1*sin(gamma)*cos(alpha)*cos(beta) - z_1*sin(beta)*cos(alpha)) + (-(-x_2 + x_3)*(z_3 - z_4) + (x_3 - x_4)*(-z_2 + z_3))*(x_1*sin(alpha)*cos(beta)*cos(gamma) - y_1*sin(alpha)*sin(gamma)*cos(beta) + z_1*sin(alpha)*sin(beta)) + ((-y_2 + y_3)*(z_3 - z_4) - (y_3 - y_4)*(-z_2 + z_3))*(-x_1*sin(beta)*cos(gamma) + y_1*sin(beta)*sin(gamma) + z_1*cos(beta));
    float j23 = (x_1*(sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha)) + y_1*(-sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)))*((-x_2 + x_3)*(y_3 - y_4) - (x_3 - x_4)*(-y_2 + y_3)) + (x_1*(-sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) + y_1*(-sin(alpha)*sin(beta)*cos(gamma) - sin(gamma)*cos(alpha)))*(-(-x_2 + x_3)*(z_3 - z_4) + (x_3 - x_4)*(-z_2 + z_3)) + ((-y_2 + y_3)*(z_3 - z_4) - (y_3 - y_4)*(-z_2 + z_3))*(-x_1*sin(gamma)*cos(beta) - y_1*cos(beta)*cos(gamma));
    float j24 = (-y_2 + y_3)*(z_3 - z_4) - (y_3 - y_4)*(-z_2 + z_3);
    float j25 = -(-x_2 + x_3)*(z_3 - z_4) + (x_3 - x_4)*(-z_2 + z_3);
    float j26 = (-x_2 + x_3)*(y_3 - y_4) - (x_3 - x_4)*(-y_2 + y_3);

    VectorXf j(6);
    j = VectorXf::Zero(6);
    j << j21, j22, j23, j24, j25, j26;
    j /= sqrt(((-x_2 + x_3)*(y_3 - y_4) - (x_3 - x_4)*(-y_2 + y_3))*((-x_2 + x_3)*(y_3 - y_4) - (x_3 - x_4)*(-y_2 + y_3)) + (-(-x_2 + x_3)*(z_3 - z_4) + (x_3 - x_4)*(-z_2 + z_3))*(-(-x_2 + x_3)*(z_3 - z_4) + (x_3 - x_4)*(-z_2 + z_3)) + ((-y_2 + y_3)*(z_3 - z_4) - (y_3 - y_4)*(-z_2 + z_3))*((-y_2 + y_3)*(z_3 - z_4) - (y_3 - y_4)*(-z_2 + z_3)));

    return j;
}
