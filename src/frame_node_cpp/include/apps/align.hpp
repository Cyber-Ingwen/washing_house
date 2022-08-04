#ifndef APPS_ALIGN_HPP_
#define APPS_ALIGN_HPP_

#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

class CloudAligner
{
    typedef pcl::PointXYZI PointT;
    public:
        CloudAligner();

        void downSample(pcl::PointCloud<PointT>::Ptr &cloud);
    private:
        pcl::PointCloud<PointT>::Ptr source_cloud;
        pcl::PointCloud<PointT>::Ptr target_cloud;

};

#endif