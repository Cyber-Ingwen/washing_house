#include "sloam/header.h"

using namespace Eigen;
using namespace std;


class MapOptmization: public rclcpp::Node
{
    /*
    创建MapOptmization节点 
    */
    public:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

        MapOptmization(std::string name);
        void odomHandler(const nav_msgs::msg::Odometry::SharedPtr msg_ptr);
};

MapOptmization::MapOptmization(std::string name): Node(name)
{
    /*创建接收和发布*/
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&MapOptmization::odomHandler, this, std::placeholders::_1));

    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("/odom2", 100);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> MapOptmization Started.\033[0m");
}

void MapOptmization::odomHandler(const nav_msgs::msg::Odometry::SharedPtr msg_ptr)
{
    nav_msgs::msg::Odometry new_msg;
    new_msg = *msg_ptr;

    new_msg.header.frame_id = "map";

    auto temp = new_msg.pose.pose.position.y;
    new_msg.pose.pose.position.x = -new_msg.pose.pose.position.x;
    new_msg.pose.pose.position.y = -temp;

    pub_odom->publish(new_msg);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapOptmization>("MapOptmization");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
