#include "sloam/header.h"

using namespace Eigen;


class LidarOdometry: public rclcpp::Node
{
    /*
    创建LidarOdometry节点 
    */
    public:

        typedef pcl::PointXYZI PointType;
    
        int isDone=1;         //标识运算是否完成
        float planeMax=0.5;   //平面判断门槛值
        std::mutex mLock;      //多线程锁
        pcl::VoxelGrid<PointType> downSizeFilterMap;  //定义点云下采样对象，用于点云抽稀
        
        /**
         * 定义ceres优化中的代价函数
         * 点到平面的距离计算用向量表示
         * 在同一坐标系中，两点间(po,pa)向量值与平面法向量(norm)的点乘可得到距离
         */
        struct PlaneFeatureCost{
            const Eigen::Vector3d _po,_pa,_norm;
        
            PlaneFeatureCost(Eigen::Vector3d po,Eigen::Vector3d pa,Eigen::Vector3d norm):
                    _po(po),_pa(pa),_norm(norm){}
        
            template <typename T>
            bool operator()(const T* q,const T* t,T* residual) const {
                Eigen::Matrix<T,3,1> po_curr{T(_po[0]),T(_po[1]),T(_po[2])};
                Eigen::Matrix<T,3,1> pa_last{T(_pa[0]),T(_pa[1]),T(_pa[2])};
                Eigen::Matrix<T,3,1> p_norm{T(_norm[0]),T(_norm[1]),T(_norm[2])};
                Eigen::Quaternion<T> q_last_curr{q[3],q[0],q[1],q[2]};   //用于坐标系变换统一
                Eigen::Matrix<T,3,1> t_last_curr{t[0],t[1],t[2]};        //用于坐标系变换统一
                Eigen::Matrix<T,3,1> po_last;
                po_last=q_last_curr*po_curr+t_last_curr;
                residual[0]=((po_last-pa_last).dot(p_norm));
                return true;
            }
        };

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_plane_frame_cloud;   //接收平面特征点云
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_plane_frame_cloud;      //发布平面特征点云
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_frame_odometry;               //发布激光雷达里程计，由帧帧配准计算得到
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_frame_odom_path;                  //发布激光雷达运动轨迹
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sum_lidar_odom_cloud;   //发布拼接后的点云地图

        //存储当前帧点云
        pcl::PointCloud<PointType>::Ptr lastFramePlanePtr = boost::make_shared<pcl::PointCloud<PointType>>();
        //存储上一帧点云
        pcl::PointCloud<PointType>::Ptr currFramePlanePtr = boost::make_shared<pcl::PointCloud<PointType>>();
        //存储拼接后总点云
        pcl::PointCloud<PointType>::Ptr sumPlaneCloudPtr = boost::make_shared<pcl::PointCloud<PointType>>();
        std::queue<sensor_msgs::msg::PointCloud2> planeQueue;   //定义点云消息队列
        nav_msgs::msg::Path lidarPathInOdom;                    //定义激光雷达运动轨迹
        std_msgs::msg::Header currHead;                         //定义ros消息头变量
        double timePlane=0;                                     //定义平面点云帧时间戳变量
        int numFrame=0;                                         //定义帧计数变量
        int flagStart=0;                                        //定义是否开始标志
        double para_q[4]={0,0,0,1};                             //定义长度为4的数组，用于构成四元数
        double para_t[3]={0,0,0};                               //定义长度为3的数组，用于构成位移
        Eigen::Quaterniond q_0_curr;                            //起点到当前帧，四元数
        Eigen::Vector3d t_0_curr;                               //起点到当前帧，位移
        Eigen::Quaterniond q_0_last;                            //起点到上一帧，四元数
        Eigen::Vector3d t_0_last;                               //起点到上一帧，位移
        
        //上一帧到当前帧，四元数
        Eigen::Quaterniond q_last_curr=Eigen::Map<Eigen::Quaterniond>(para_q);
        //上一帧到当前帧，位移
        Eigen::Vector3d t_last_curr=Eigen::Map<Eigen::Vector3d>(para_t);

        LidarOdometry(std::string name): Node(name)
        {
            /*创建并初始化接收*/
            sub_plane_frame_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>("/plane_frame_cloud1", 10, std::bind(&LidarOdometry::cloudHandler, this, std::placeholders::_1));
            
            pub_plane_frame_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/plane_frame_cloud2", 100);
            pub_frame_odometry = this->create_publisher<nav_msgs::msg::Odometry>("/frame_odom2", 100);
            pub_frame_odom_path = this->create_publisher<nav_msgs::msg::Path>("/frame_odom_path2", 100);
            pub_sum_lidar_odom_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sum_lidar_odom_cloud2", 10);

            Eigen::Quaterniond temp(0,0,0,1); 
            q_0_curr = temp;     
            q_0_last = temp;                
            t_0_curr << 0,0,0;  
            t_0_last << 0,0,0;  
            downSizeFilterMap.setLeafSize(0.4,0.4,0.4);

            RCLCPP_INFO(this->get_logger(), "\033[1;32m----> LidarOdometry Started.\033[0m");
        }

        void transformToLast(PointType const *const pi,PointType *const po);
        void publishResult();
        void frameRegistration();
        void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cldMsg);
        void cloudThread();
};

void LidarOdometry::transformToLast(PointType const *const pi,PointType *const po)
{
    Eigen::Vector3d curr_point(pi->x,pi->y,pi->z);
    Eigen::Vector3d proj_point;
    proj_point=q_last_curr*curr_point+t_last_curr;
    po->x=proj_point.x();
    po->y=proj_point.y();
    po->z=proj_point.z();
    po->intensity=pi->intensity;
}

//发布点云、里程计、轨迹、地图
void LidarOdometry::publishResult(){
    //累计帧间变换，得到从起点开始的里程计
    q_0_curr = q_0_last * q_last_curr ;
    t_0_curr = t_0_last + q_0_last * t_last_curr;
    q_0_last = q_0_curr;
    t_0_last = t_0_curr;
    //发布里程计
    nav_msgs::msg::Odometry LO;
    LO.header.frame_id = "map";
    LO.child_frame_id = "map_child";
    LO.header.stamp = currHead.stamp;
    LO.pose.pose.position.x = t_0_curr[0];
    LO.pose.pose.position.y = t_0_curr[1];
    LO.pose.pose.position.z = t_0_curr[2];
    LO.pose.pose.orientation.x = q_0_curr.x();
    LO.pose.pose.orientation.y = q_0_curr.y();
    LO.pose.pose.orientation.z = q_0_curr.z();
    LO.pose.pose.orientation.w = q_0_curr.w();
    pub_frame_odometry->publish(LO);
    //发布里轨迹
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = LO.header.stamp;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose = LO.pose.pose;
    lidarPathInOdom.poses.push_back(pose_stamped);
    lidarPathInOdom.header.stamp=LO.header.stamp;
    lidarPathInOdom.header.frame_id="map";
    pub_frame_odom_path->publish(lidarPathInOdom);
    //发布平面特征点云
    sensor_msgs::msg::PointCloud2 plane_frame_cloud_msgs;
    pcl::toROSMsg(*currFramePlanePtr, plane_frame_cloud_msgs);
    plane_frame_cloud_msgs.header.stamp = LO.header.stamp;
    plane_frame_cloud_msgs.header.frame_id = "map";
    pub_plane_frame_cloud->publish(plane_frame_cloud_msgs);
    //发布拼接点云地图
    if(numFrame % 5 == 0)
    {
        double r,p,y;
        tf2::Quaternion tq(q_0_curr.x(),q_0_curr.y(),q_0_curr.z(),q_0_curr.w());
        tf2::Matrix3x3(tq).getRPY(r,p,y);
        Eigen::Affine3d transCurd;
        pcl::getTransformation(t_0_curr.x(), t_0_curr.y(), t_0_curr.z(), r,p,y,transCurd);
        pcl::PointCloud<PointType>::Ptr cloud_res = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl::transformPointCloud(*currFramePlanePtr, *cloud_res, transCurd);
        *sumPlaneCloudPtr += *cloud_res;

        pcl::PointCloud<PointType>::Ptr cloud_temp = boost::make_shared<pcl::PointCloud<PointType>>();
        downSizeFilterMap.setInputCloud(sumPlaneCloudPtr);
        downSizeFilterMap.filter(*cloud_temp);

        sensor_msgs::msg::PointCloud2 res_cloud_msgs;
        pcl::toROSMsg(*cloud_temp, res_cloud_msgs);
        res_cloud_msgs.header.stamp = LO.header.stamp;
        res_cloud_msgs.header.frame_id = "map";
        pub_sum_lidar_odom_cloud->publish(res_cloud_msgs);
   }
}

//计算帧帧配准，得到帧间位姿变换
void LidarOdometry::frameRegistration(){
    //定义ceres优化对象与参数
    ceres::LossFunction *loss_function=new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization=new ceres::EigenQuaternionParameterization();
    ceres::Problem problem;
    problem.AddParameterBlock(para_q,4,q_parameterization);
    problem.AddParameterBlock(para_t,3);
 
    pcl::KdTreeFLANN<PointType> kdTreePlanLast;
    int last_plane_num=lastFramePlanePtr->points.size();
    int curr_plane_num=currFramePlanePtr->points.size();
    if(last_plane_num>10){
        kdTreePlanLast.setInputCloud(lastFramePlanePtr);
        for(int i_opt=0;i_opt<2;i_opt++){
            for (int i = 0; i < curr_plane_num; ++i) {    //遍历当前帧各平面点
                PointType pointSeed;
                //将当前帧此平面点坐标变换到上一帧坐标系中
                transformToLast(&currFramePlanePtr->points[i],&pointSeed);
                std::vector<float> pointSearchSqDis1;
                std::vector<int> indx1;
                //将变换后的此点作为种子点，查找上一帧中距离此点最近点的索引
                kdTreePlanLast.nearestKSearch(pointSeed,1,indx1,pointSearchSqDis1);
                int p_ind_a=indx1[0];
                std::vector<float> pointSearchSqDis2;
                std::vector<int> indx2;
                //将上面最近点作为种子点，查找上一帧中距离最近点的索引和距离
                kdTreePlanLast.nearestKSearch(lastFramePlanePtr->points[p_ind_a],30,indx2,pointSearchSqDis2);
                std::vector<int> v_indx5;
                std::vector<int> v_indx_row;
                int p_row=-1;
                if(indx2.size()<5) continue;
                int n=5;
                //挑选5个最近点，尽量满足有2个点不属于同一扫描线
                for (size_t i_kd = 0; i_kd < indx2.size(); ++i_kd) {
                    float f_indx=lastFramePlanePtr->points[indx2[i_kd]].intensity;
                    int i_indx=int(f_indx);
                    int row=100*(f_indx-i_indx+0.002);   //获取点索引
                    if(i_kd==0){
                        p_row=row;
                    }
                    if(i_kd<5){       //先将最近5个点选入
                        v_indx5.push_back(indx2[i_kd]);
                    }else{            //从第6个点开始，寻找与记录不同线的最近2个点
                        if(row != p_row && row>=0 && row <=63){
                            v_indx_row.push_back(indx2[i_kd]);
                            n=i_kd;
                            if(v_indx_row.size()>=2){
                                break;
                            }
                        }
                    }
                }
                if(v_indx_row.size()==1){       //如果不同线的只有1个点
                    v_indx5[4]=v_indx_row[0];
                }
                if(v_indx_row.size()==2){       //如果不同线的有2个点
                    v_indx5[3]=v_indx_row[0];
                    v_indx5[4]=v_indx_row[1];
                }
 
                if(pointSearchSqDis2[n]<1){     //如果5个点中最远的点小于1米，则利用5点估算平面法线
                    Eigen::Matrix<float, 5, 3> matA0;
                    Eigen::Matrix<float, 5, 1> matB0;
                    Eigen::Vector3f matX0;
                    matA0.setZero();
                    matB0.fill(-1);
                    matX0.setZero();
                    for (int j = 0; j < 5; ++j) {
                        matA0(j,0)=lastFramePlanePtr->points[v_indx5[j]].x;
                        matA0(j,1)=lastFramePlanePtr->points[v_indx5[j]].y;
                        matA0(j,2)=lastFramePlanePtr->points[v_indx5[j]].z;
                    }
                    matX0=matA0.colPivHouseholderQr().solve(matB0);
                    matX0.normalize();  //norm
                    bool planeValid = true;
                    for (int k = 0; k < 4; ++k) {   //利用法向量计算各点到平面的距离
                        Eigen::Vector3d v_temp(
                                lastFramePlanePtr->points[v_indx5[k]].x-lastFramePlanePtr->points[v_indx5[k+1]].x,
                                lastFramePlanePtr->points[v_indx5[k]].y-lastFramePlanePtr->points[v_indx5[k+1]].y,
                                lastFramePlanePtr->points[v_indx5[k]].z-lastFramePlanePtr->points[v_indx5[k+1]].z
                        );
                        if(fabs(matX0(0)*v_temp[0]+matX0(1)*v_temp[1]+matX0(2)*v_temp[2])>planeMax){
                            planeValid=false;       //如果有点到平面的距离太大，则说明此5点不共面
                            break;
                        }
                    }
                    Eigen::Vector3d po(currFramePlanePtr->points[i].x,currFramePlanePtr->points[i].y,currFramePlanePtr->points[i].z);
                    Eigen::Vector3d pa(lastFramePlanePtr->points[p_ind_a].x,lastFramePlanePtr->points[p_ind_a].y,lastFramePlanePtr->points[p_ind_a].z);
                    Eigen::Vector3d norm(matX0[0],matX0[1],matX0[2]);
                    if(planeValid){                 //当找到了共面点，就利用种子点、最近点、平面法向量，构造点与平面共面的优化条件
                        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<PlaneFeatureCost,1,4,3>
                                                         (new PlaneFeatureCost(po,pa,norm)),loss_function,para_q,para_t);
                    }
                }
            }
        }
        //设置优化参数，并优化
        ceres::Solver::Options options;
        options.linear_solver_type=ceres::DENSE_QR;
        options.max_num_iterations=8;
        options.minimizer_progress_to_stdout=false;
        ceres::Solver::Summary summary;
        ceres::Solve(options,&problem,&summary);
        //得到优化结果，构成帧间变换的方向四元数与位移
        q_last_curr=Eigen::Map<Eigen::Quaterniond>(para_q);
        t_last_curr=Eigen::Map<Eigen::Vector3d>(para_t);
    }
}

//接收平面特征点云，添加到消息队列中
void LidarOdometry::cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cldMsg) {
    mLock.lock();
    planeQueue.push(*cldMsg);
    mLock.unlock();
}

//点云处理多线程函数
void LidarOdometry::cloudThread()
{
    rclcpp::WallRate loop_rate(10);                 //控制处理频率为10hz，为最终频率
    while(rclcpp::ok())
    {
        loop_rate.sleep();
        rclcpp::WallRate loop_rate2(50);            //内层处理频率为50hz
        if(isDone==0) continue;
        while (planeQueue.size()>0)
        {
            if(isDone==0) continue;
            isDone=0;
            numFrame++;
            loop_rate2.sleep();
 
            mLock.lock();                           //锁线程，取数据
            currFramePlanePtr->clear();
            currHead=planeQueue.front().header;
            timePlane=double(planeQueue.front().header.stamp.sec) + 1e-9 * double(planeQueue.front().header.stamp.nanosec);
            pcl::fromROSMsg(planeQueue.front(),*currFramePlanePtr);
            planeQueue.pop();
            mLock.unlock();
 
            if(flagStart==0)
            {
                flagStart=1;
            }
            else
            {         
                this->frameRegistration();
                this->publishResult();
            }
            *lastFramePlanePtr=*currFramePlanePtr;
            isDone=1;
        }
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarOdometry>("LidarOdometry");

    std::thread t_cloud_thread(&LidarOdometry::cloudThread, node);
    
    rclcpp::spin(node);
    t_cloud_thread.join();
    rclcpp::shutdown();
    
    return 0;
}
