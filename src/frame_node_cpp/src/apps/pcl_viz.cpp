#include "apps/pcl_viz.hpp"

CloudVisualizer::CloudVisualizer(std::string name)
{
    viewer = boost::make_shared<pcl::visualization::PCLVisualizer>(name);
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(name));
    viewer->setBackgroundColor(100, 100, 100);

}

// template<typename T>
// void CloudVisualizer::VisualUpdate(typename pcl::PointCloud<T>::Ptr cloud, std::string name)
void CloudVisualizer::VisualUpdate(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string name)
{
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity(cloud, "intensity");
    viewer->removeAllPointClouds();  // 移除当前所有点云
    viewer->addPointCloud(cloud, intensity, name, 0);
    // viewer->updatePointCloud(cloud, name);
    viewer->spinOnce(0.001);
}