#ifndef PCL_VISUALIZER_HPP_
#define PCL_VISUALIZER_HPP_s

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

class CloudVisualizer
{
    public:
        CloudVisualizer(std::string name);

        // template<typename T>
        // void VisualUpdate(typename pcl::PointCloud<T>::Ptr cloud, std::string name);
        void VisualUpdate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string name);

    private:
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

};

#endif