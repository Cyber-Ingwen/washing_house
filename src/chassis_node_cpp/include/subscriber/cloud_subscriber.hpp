#ifndef SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_

#include <iostream>
#include <deque>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "apps/pcl_viz.hpp"

class CloudSubscriber : public rclcpp::Node
{
  public:
    CloudSubscriber(std::string name, std::string topic_name);

  private:
    std::string viz_name;
    boost::shared_ptr<CloudVisualizer> visualizer;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    void msg_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);

    template<typename T>
    void fromROSMsg(const sensor_msgs::msg::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud);

    void copyPointCloud2MetaData(const sensor_msgs::msg::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2);
    void toPCL(const sensor_msgs::msg::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2);
    void toPCL(const std_msgs::msg::Header &header, pcl::PCLHeader &pcl_header);
    void toPCL(const rclcpp::Time &stamp, std::uint64_t &pcl_stamp);
    void toPCL(const sensor_msgs::msg::PointField &pf, pcl::PCLPointField &pcl_pf);
    void toPCL(const std::vector<sensor_msgs::msg::PointField> &pfs, std::vector<pcl::PCLPointField> &pcl_pfs);

};

#endif