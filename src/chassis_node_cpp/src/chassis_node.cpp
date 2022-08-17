#include "rclcpp/rclcpp.hpp"
#include <pcl/common/transforms.h>
// #include <open3d/Open3D.h>

#include "subscriber/cloud_subscriber.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto node1 = std::make_shared<CloudSubscriber>("cloud_sub","/kitti/velo/pointcloud");

    exec.add_node(node1);
    
    exec.spin();
    rclcpp::shutdown();
    return 0;
}