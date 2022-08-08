#include "apps/align.hpp"
#include <pcl/visualization/pcl_visualizer.h>


CloudAligner::CloudAligner()
{
    source_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
    // target_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
    ndt_omp = boost::make_shared<pclomp::NormalDistributionsTransform<PointT, PointT>>();
    ndt_omp->setResolution(1.0);

    vis = boost::make_shared<pcl::visualization::PCLVisualizer>("ndt vis");

}

void CloudAligner::downSample(pcl::PointCloud<PointT>::Ptr &cloud)
{
    // pcl::PointCloud<PointT>::Ptr downsampled = boost::make_shared<pcl::PointCloud<PointT>>();

    pcl::VoxelGrid<PointT> voxelgrid;
    voxelgrid.setLeafSize(0.5f,0.5f,0.5f);
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*cloud);
    source_cloud = cloud;

    // *cloud = *downsampled;
}

void CloudAligner::ndtAlign()
{
    if (target_cloud == nullptr)
    {
        target_cloud = source_cloud;

        std::cout << "Yi! Wu!" << std::endl;
        return;
    }
        // std::vector<int> num_threads = {1, omp_get_max_threads()};
        // std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>> search_methods = {
        //     {"KDTREE", pclomp::KDTREE},
        //     {"DIRECT7", pclomp::DIRECT7},
        //     {"DIRECT1", pclomp::DIRECT1}
        // };
        // for(int n : num_threads) {
        //     for(const auto& search_method : search_methods) {
        //         std::cout << "--- pclomp::NDT (" << search_method.first << ", " << n << " threads) ---" << std::endl;
        //         ndt_omp->setNumThreads(n);
        //         ndt_omp->setNeighborhoodSearchMethod(search_method.second);
        //         aligned = align(ndt_omp, target_cloud, source_cloud);
        //     }
        // }
    std::cout << std::endl <<  "--- pclomp::NDT (DIRECT7, " << omp_get_max_threads() << " threads) ---" << std::endl;
    ndt_omp->setNumThreads(omp_get_max_threads());
    ndt_omp->setNeighborhoodSearchMethod(pclomp::KDTREE);
    aligned = align(ndt_omp, target_cloud, source_cloud);

    vis->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerCustom<PointT> target_handler(target_cloud, 255.0, 0.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> source_handler(source_cloud, 0.0, 255.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> aligned_handler(aligned, 0.0, 0.0, 255.0);

    // vis->addPointCloud(target_cloud, target_handler, "target");
    // vis->addPointCloud(source_cloud, source_handler, "source");
    // vis->addPointCloud(aligned, aligned_handler, "aligned");
    vis->addPointCloud(aligned, source_handler, "aligned");
    vis->spinOnce(0.001);

    target_cloud = source_cloud;
}

pcl::PointCloud<PointT>::Ptr CloudAligner::align(pcl::Registration<PointT, PointT>::Ptr registration, const pcl::PointCloud<PointT>::Ptr& target_cloud, const pcl::PointCloud<PointT>::Ptr& source_cloud ) {
  registration->setInputTarget(target_cloud);
  registration->setInputSource(source_cloud);
  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());

//   auto t1 = rclcpp::Clock().now();
//   registration->align(*aligned);
  auto t2 = rclcpp::Clock().now();
//   std::cout << "single : " << (t2 - t1).seconds() * 1000 << "[msec]" << std::endl;

  for(int i=0; i<10; i++) {
    registration->align(*aligned);
    if (registration->getFitnessScore() < .4f) break;
  }
  auto t3 = rclcpp::Clock().now();
  std::cout << "times: " << (t3 - t2).seconds() * 1000 << "[msec]" << std::endl;
  std::cout << "fitness: " << registration->getFitnessScore() << std::endl << std::endl;

  return aligned;
}

