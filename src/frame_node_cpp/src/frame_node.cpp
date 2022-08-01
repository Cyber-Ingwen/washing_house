#include "rclcpp/rclcpp.hpp"
#include <pcl/common/transforms.h>
// #include <open3d/Open3D.h>

#include "subscriber/cloud_subscriber.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudSubscriber>("cloud_sub","/kitti/velo/pointcloud");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}