#include "subscriber/cloud_subscriber.hpp"
#include <charconv>

// #include "glog/logging.h"

CloudSubscriber::CloudSubscriber(std::string name, std::string topic_name) : Node(name)
{
    viz_name = "pcl cloud";
    visualizer = boost::make_shared<CloudVisualizer>(viz_name);

    aligner = boost::make_shared<CloudAligner>();

    subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 10, std::bind(&CloudSubscriber::msg_callback, this, std::placeholders::_1));
}

void CloudSubscriber::msg_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr )
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    CloudSubscriber::fromROSMsg(*msg_ptr, *cloud);
    // RCLCPP_INFO(this->get_logger(), "succuse!");

    // std::string s = std::to_string(cloud->width);
    // char const *pchar = s.c_str();
    // RCLCPP_INFO(this->get_logger(), "CloudSize: ");
    // RCLCPP_INFO(this->get_logger(), pchar);
    aligner->downSample(cloud);
    std::string s = std::to_string(cloud->width);
    char const *pchar = s.c_str();
    RCLCPP_INFO(this->get_logger(), "CloudSize: ");
    RCLCPP_INFO(this->get_logger(), pchar);

    visualizer->visualUpdate(cloud, viz_name);
};

template<typename T>
void CloudSubscriber::fromROSMsg(const sensor_msgs::msg::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud)
{
    pcl::PCLPointCloud2 pcl_pc2;
    CloudSubscriber::toPCL(cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
}

void CloudSubscriber::toPCL(const sensor_msgs::msg::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
{
    copyPointCloud2MetaData(pc2, pcl_pc2);
    pcl_pc2.data = pc2.data;
}

void CloudSubscriber::copyPointCloud2MetaData(const sensor_msgs::msg::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
{
    CloudSubscriber::toPCL(pc2.header, pcl_pc2.header);
    pcl_pc2.height = pc2.height;
    pcl_pc2.width = pc2.width;
    CloudSubscriber::toPCL(pc2.fields, pcl_pc2.fields);
    pcl_pc2.is_bigendian = pc2.is_bigendian;
    pcl_pc2.point_step = pc2.point_step;
    pcl_pc2.row_step = pc2.row_step;
    pcl_pc2.is_dense = pc2.is_dense;
}

void CloudSubscriber::toPCL(const std_msgs::msg::Header &header, pcl::PCLHeader &pcl_header)
{
    CloudSubscriber::toPCL(header.stamp, pcl_header.stamp);
    // TODO(clalancette): Seq doesn't exist in the ROS2 header
    // anymore.  wjwwood suggests that we might be able to get this
    // information from the middleware in the future, but for now we
    // just set it to 0.
    pcl_header.seq = 0;
    pcl_header.frame_id = header.frame_id;
}

void CloudSubscriber::toPCL(const rclcpp::Time &stamp, std::uint64_t &pcl_stamp)
{
    pcl_stamp = stamp.nanoseconds() / 1000ull;  // Convert from ns to us
}

void CloudSubscriber::toPCL(const sensor_msgs::msg::PointField &pf, pcl::PCLPointField &pcl_pf)
{   
    if (pf.name=="i")
    {
        pcl_pf.name = "intensity";
    }
    else
    {
        pcl_pf.name = pf.name; 
    }
    pcl_pf.offset = pf.offset;
    pcl_pf.datatype = pf.datatype;
    pcl_pf.count = pf.count;
}

void CloudSubscriber::toPCL(const std::vector<sensor_msgs::msg::PointField> &pfs, std::vector<pcl::PCLPointField> &pcl_pfs)
{
    pcl_pfs.resize(pfs.size());
    std::vector<sensor_msgs::msg::PointField>::const_iterator it = pfs.begin();
    int i = 0;
    for(; it != pfs.end(); ++it, ++i) {
      CloudSubscriber::toPCL(*(it), pcl_pfs[i]);
    }
}



