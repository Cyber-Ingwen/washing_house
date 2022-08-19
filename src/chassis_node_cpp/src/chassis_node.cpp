#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/common/transforms.h>

#include "subscriber/cloud_subscriber.hpp"
// #include "hunter/hunter_base_ros.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    // auto robot = std::make_shared<HunterBaseRos>("hunter");
    // robot->Initialize();
    std::cout << "Robot initialized, start running ..." << std::endl;
    
    auto test_node = std::make_shared<CloudSubscriber>("cloud_sub","/kitti/velo/pointcloud");
    exec.add_node(test_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}