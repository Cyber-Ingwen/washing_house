#ifndef APPS_ALIGN_HPP_
#define APPS_ALIGN_HPP_

#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

typedef pcl::PointXYZI PointT;

class CloudAligner
{
    public:
        CloudAligner();
        void downSample(pcl::PointCloud<PointT>::Ptr &cloud);
        void ndtAlign();
    private:
        boost::shared_ptr<pcl::visualization::PCLVisualizer> vis;

        pcl::PointCloud<PointT>::Ptr source_cloud;
        pcl::PointCloud<PointT>::Ptr target_cloud;
        pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt_omp;
        pcl::PointCloud<PointT>::Ptr aligned;

        pcl::PointCloud<PointT>::Ptr align(pcl::Registration<PointT, PointT>::Ptr registration, const pcl::PointCloud<PointT>::Ptr& target_cloud, const pcl::PointCloud<PointT>::Ptr& source_cloud );

};

#endif