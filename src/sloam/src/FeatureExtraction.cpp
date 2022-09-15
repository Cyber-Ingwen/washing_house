#include "sloam/header.h"

using namespace Eigen;
using namespace std;


class FeatureExtraction: public rclcpp::Node
{
    /*
    创建FeatureExtraction节点 
    */
    public:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_edge_points;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_plane_points;
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcn;
        pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points;
        pcl::PointCloud<pcl::PointXYZI>::Ptr plane_points;

        std::string topic_name;

        FeatureExtraction(std::string name);
        void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);
        void feature_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
        void publish_features(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr);
};

FeatureExtraction::FeatureExtraction(std::string name): Node(name, 
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
    this->get_parameter("topic_name",topic_name);
    /*创建接收和发布点云*/
    // topic_name = "/rslidar_points";
    sub_point_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 100, std::bind(&FeatureExtraction::cloudHandler, this, std::placeholders::_1));

    pub_edge_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/edge_points", 100);
    pub_plane_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/plane_points", 100);

    edge_points = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    plane_points = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcn = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> FeatureExtraction Started.\033[0m");
}

void FeatureExtraction::cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*msg_ptr, *cloud);

    this->feature_extraction(cloud);
    this->publish_features(msg_ptr);
}

void FeatureExtraction::feature_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcn->clear();
    edge_points->clear();
    plane_points->clear();

    /* 分割地面点 */
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if(i % 16 >= 4)
        {
            pcn->points.push_back(cloud->points[i]);
        }
    }

    /* 提取竖线和平面 */
    for (int sector = 0; sector < 6; sector++)
    {
        vector<float> curv_list;
        for (int i = int(pcn->points.size() / 6) * sector; i < int(pcn->points.size() / 6) * (sector + 1); i++)
        {
            float x = pcn->points[i].x;
            float y = pcn->points[i].y;
            float z = pcn->points[i].z;
            float r0 = x*x + y*y + z*z;

            float curv = 0;
            float sum[3] = {0, 0, 0};
            float sum2 = 0;

            if((i - 12 * 5 >= 0 ) && (i + 12 * 5 < pcn->points.size()))
            {
                for (int j = 0; j < 5; j++)
                {
                    int next_index = i + 12 * j;
                    int last_index = i - 12 * j;
                    sum[0] += (x - pcn->points[last_index].x);
                    sum[0] += (x - pcn->points[next_index].x);
                    sum[1] += (y - pcn->points[last_index].y);
                    sum[1] += (y - pcn->points[next_index].y);
                    sum[2] += (z - pcn->points[last_index].z);
                    sum[2] += (z - pcn->points[next_index].z);

                    sum2 += (x - pcn->points[last_index].x) * (x - pcn->points[last_index].x) + (y - pcn->points[last_index].y) * (y - pcn->points[last_index].y) + (z - pcn->points[last_index].z) * (z - pcn->points[last_index].z);
                    sum2 += (x - pcn->points[next_index].x) * (x - pcn->points[next_index].x) + (y - pcn->points[next_index].y) * (y - pcn->points[next_index].y) + (z - pcn->points[next_index].z) * (z - pcn->points[next_index].z);
                }

                curv = (sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]) / (sum2);
                if (isnan(curv)){ curv = -1;}
                if (r0 < 10){ curv = -1;}

                int next_index = i + 12 * 5;
                int last_index = i - 12 * 5;
                float rl = pcn->points[last_index].x * pcn->points[last_index].x + pcn->points[last_index].y * pcn->points[last_index].y + pcn->points[last_index].z * pcn->points[last_index].z;
                float rn = pcn->points[next_index].x * pcn->points[next_index].x + pcn->points[next_index].y * pcn->points[next_index].y + pcn->points[next_index].z * pcn->points[next_index].z;
                if ((abs(rl - r0) / r0 > 0.2) || (abs(rn - r0) / r0 > 0.2))
                {
                    curv = -1;
                }

                curv_list.push_back(curv);
            }
        }
        
        vector<int> index(curv_list.size());
        for (int ind = 0; ind < curv_list.size(); ind++) {index[ind] = ind;}
        auto rule = [curv_list](int a, int b) -> bool{return curv_list[a] < curv_list[b];};
        sort(index.begin(), index.end(), rule);

        vector<int> plane_index; 
        vector<int> edge_index;

        for (int j = 0; j < index.size(); j++)
        {
            if (plane_index.size() >= 3000)
            {
                break;
            }
            if (curv_list[index[j]] > 0)
            {
                int flag = 1;
                for (int k = 0; k < 5; k++)
                {
                    if (index[j] + k < curv_list.size())
                    {
                        if (curv_list[index[j]] > curv_list[index[j] + k]) {flag = 0;}
                    }
                    if (index[j] - k > 0)
                    {
                        if (curv_list[index[j]] > curv_list[index[j] - k]) {flag = 0;}
                    }
                }
                if (flag == 1)
                {
                    plane_index.push_back(index[j]);
                }
            }
        }

        reverse(index.begin(), index.end());
        for (int j = 0; j < index.size(); j++)
        {
            if (edge_index.size() >= 48)
            {
                break;
            }
            if ((curv_list[index[j]] < 100) && (curv_list[index[j]] > 0))
            {
                int flag = 1;
                for (int k = 0; k < 3; k++)
                {
                    if (index[j] + k < curv_list.size())
                    {
                        if (curv_list[index[j]] < curv_list[index[j] + k]) {flag = 0;}
                    }
                    if (index[j] - k > 0)
                    {
                        if (curv_list[index[j]] < curv_list[index[j] - k]) {flag = 0;}
                    }
                }
                if (flag == 1)
                {
                    edge_index.push_back(index[j]);
                }
            }
        }

        for (int ind : plane_index)
        {
            ind += int(pcn->points.size() / 6) * sector;
            if (sector == 0)
            {
                ind += 12 * 5;
            }
            pcn->points[ind].intensity = ind;
            plane_points->points.push_back(pcn->points[ind]);
        }

        for (int ind : edge_index)
        {
            ind += int(pcn->points.size() / 6) * sector;
            if (sector == 0)
            {
                ind += 12 * 5;
            }
            pcn->points[ind].intensity = ind;
            edge_points->points.push_back(pcn->points[ind]);
        }
    }
}

void FeatureExtraction::publish_features(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr)
{
    //发布边缘点
    sensor_msgs::msg::PointCloud2 edge_points_msg_ptr;
    pcl::toROSMsg(*edge_points, edge_points_msg_ptr);
    edge_points_msg_ptr.header.stamp = msg_ptr->header.stamp;
    edge_points_msg_ptr.header.frame_id = "odom";
    pub_edge_points->publish(edge_points_msg_ptr);

    //发布平面点
    sensor_msgs::msg::PointCloud2 plane_points_msg_ptr;
    pcl::toROSMsg(*plane_points, plane_points_msg_ptr);
    plane_points_msg_ptr.header.stamp = msg_ptr->header.stamp;
    plane_points_msg_ptr.header.frame_id = "odom";
    pub_plane_points->publish(plane_points_msg_ptr);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureExtraction>("FeatureExtraction");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
