#include "apps/align.hpp"

CloudAligner::CloudAligner()
{
    source_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
    target_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
    ndt = boost::make_shared<pcl::NormalDistributionsTransform<PointT,PointT>>();
    ndt->setResolution(1.0);
}

void CloudAligner::downSample(pcl::PointCloud<PointT>::Ptr &cloud)
{
    // pcl::PointCloud<PointT>::Ptr downsampled = boost::make_shared<pcl::PointCloud<PointT>>();

    pcl::VoxelGrid<PointT> voxelgrid;
    voxelgrid.setLeafSize(0.3f,0.3f,0.3f);
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*cloud);
    // *cloud = *downsampled;
}

void CloudAligner::ndtAlign()
{
    return;
}

