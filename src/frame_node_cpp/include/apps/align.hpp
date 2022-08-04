#ifndef APPS_ALIGN_HPP_
#define APPS_ALIGN_HPP_

#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pclomp/ndt_omp.h>

class CloudAligner
{
    typedef pcl::PointXYZI PointT;
    public:
        CloudAligner();
        void downSample(pcl::PointCloud<PointT>::Ptr &cloud);
        void ndtAlign();
    private:
        pcl::PointCloud<PointT>::Ptr source_cloud;
        pcl::PointCloud<PointT>::Ptr target_cloud;
        pcl::NormalDistributionsTransform<PointT, PointT>::Ptr ndt;

        pcl::PointCloud<PointT>::Ptr align(pcl::Registration<PointT, PointT>::Ptr registration, const pcl::PointCloud<PointT>::Ptr& target_cloud, const pcl::PointCloud<PointT>::Ptr& source_cloud );

};

#endif