/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "visualization.h"

// 使用命名空间，简化代码编写
using namespace ros;     // ROS相关
using namespace Eigen;   // Eigen数学库

// ==================== 全局发布器定义 ====================

// 里程计发布器：发布优化后的完整里程计信息
ros::Publisher pub_odometry;
// 最新里程计发布器：发布IMU传播的实时里程计（高频）
ros::Publisher pub_latest_odometry;

// 路径发布器：发布历史轨迹路径
ros::Publisher pub_path;

// 点云发布器：发布当前观测到的点云
ros::Publisher pub_point_cloud;
// 边缘化点云发布器：发布被边缘化的点云（用于可视化）
ros::Publisher pub_margin_cloud;

// 关键帧位姿发布器：发布关键帧位姿（用于可视化）
ros::Publisher pub_key_poses;

// 相机位姿发布器：发布相机坐标系下的位姿
ros::Publisher pub_camera_pose;
// 相机位姿可视化发布器：发布用于RViz显示的MarkerArray
ros::Publisher pub_camera_pose_visual;

// 路径消息对象：存储历史位姿，构成连续路径
nav_msgs::Path path;

// 关键帧位姿发布器：发布关键帧的完整位姿信息
ros::Publisher pub_keyframe_pose;
// 关键帧点云发布器：发布关键帧关联的点云
ros::Publisher pub_keyframe_point;

// 外参发布器：发布相机到IMU的外参变换
ros::Publisher pub_extrinsic;

// 图像跟踪结果发布器：发布带特征点跟踪的可视化图像
ros::Publisher pub_image_track;

// 相机位姿可视化对象：用于生成相机模型的可视化Marker
// 参数：红色(1,0,0,1)
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);

// 路径长度统计：累计轨迹总长度
static double sum_of_path = 0;
// 上一帧位置：用于计算路径增量
static Vector3d last_path(0.0, 0.0, 0.0);

// 发布计数器：统计发布次数（可能用于控制发布频率）
size_t pub_counter = 0;

// ==================== 注册所有发布器 ====================
// 功能：创建所有ROS发布器并指定话题名称
// 参数：n - ROS节点句柄
void registerPub(ros::NodeHandle &n)
{
    // 1. 发布IMU传播的实时里程计（高频）
    // 话题名：imu_propagate
    // 队列大小：1000
    // 消息类型：nav_msgs::Odometry
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    
    // 2. 发布历史轨迹路径
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    
    // 3. 发布优化后的里程计（后端优化结果）
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    
    // 4. 发布当前帧观测到的点云
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
    
    // 5. 发布被边缘化的点云（可视化用）
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud", 1000);
    
    // 6. 发布关键帧位姿（Marker形式）
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    
    // 7. 发布相机位姿（相机坐标系）
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    
    // 8. 发布相机位姿可视化（MarkerArray形式）
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    
    // 9. 发布关键帧位姿（完整信息）
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    
    // 10. 发布关键帧关联的点云
    pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
    
    // 11. 发布相机-IMU外参
    pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
    
    // 12. 发布特征跟踪图像
    pub_image_track = n.advertise<sensor_msgs::Image>("image_track", 1000);

    // 设置相机位姿可视化参数
    cameraposevisual.setScale(0.1);      // 相机模型大小
    cameraposevisual.setLineWidth(0.01); // 线条宽度
}

// ==================== 发布最新里程计 ====================
// 功能：发布IMU传播得到的实时里程计信息（高频、低延迟）
// 注意：这里发布的是未经后端优化的结果，用于实时性要求高的场景
// 参数：
//   P: 位置 (x, y, z)
//   Q: 旋转（四元数）
//   V: 速度 (vx, vy, vz)
//   t: 时间戳
void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, 
                       const Eigen::Vector3d &V, double t)
{
    // 创建Odometry消息
    nav_msgs::Odometry odometry;
    
    // 设置消息头
    odometry.header.stamp = ros::Time(t);  // 时间戳
    odometry.header.frame_id = "world";    // 父坐标系
    
    // 设置位置
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    
    // 设置姿态（四元数）
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    
    // 设置速度
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    
    // 发布消息
    pub_latest_odometry.publish(odometry);
}

// ==================== 发布跟踪图像 ====================
// 功能：将带有特征点跟踪标记的图像发布为ROS图像消息
// 参数：
//   imgTrack: OpenCV图像（已绘制跟踪结果）
//   t: 时间戳
void pubTrackImage(const cv::Mat &imgTrack, const double t)
{
    // 创建消息头
    std_msgs::Header header;
    header.frame_id = "world";          // 坐标系（虽然图像不依赖坐标系，但需要设置）
    header.stamp = ros::Time(t);       // 时间戳
    
    // 使用cv_bridge将OpenCV图像转换为ROS图像消息
    // "bgr8"表示图像编码为BGR格式，8位每通道
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    
    // 发布图像
    pub_image_track.publish(imgTrackMsg);
}

// ==================== 打印统计信息 ====================
// 功能：输出调试信息和统计结果，用于性能分析和监控
// 参数：
//   estimator: 估计器对象（包含所有状态估计）
//   t: 处理当前帧花费的时间（毫秒）
void printStatistics(const Estimator &estimator, double t)
{
    // 如果系统还处于初始化阶段，不打印统计信息
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    
    // 调试输出当前位置
    // printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    
    // 调试输出当前速度
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    
    // 如果系统配置了在线估计外参
    if (ESTIMATE_EXTRINSIC)
    {
        // 创建OpenCV文件存储器，用于保存标定结果到YAML文件
        cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
        
        // 遍历所有相机（通常是双目：0左目，1右目）
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            // 输出外参平移和旋转（旋转以欧拉角显示）
            ROS_DEBUG_STREAM("extrinsic tic: " << estimator.tic[i].transpose());
            ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());

            // 构建4x4变换矩阵 [R | t; 0 0 0 1]
            Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
            eigen_T.block<3, 3>(0, 0) = estimator.ric[i];  // 旋转部分
            eigen_T.block<3, 1>(0, 3) = estimator.tic[i];  // 平移部分
            
            // 转换为OpenCV矩阵格式
            cv::Mat cv_T;
            cv::eigen2cv(eigen_T, cv_T);
            
            // 保存到YAML文件，不同相机使用不同的标签
            if(i == 0)
                fs << "body_T_cam0" << cv_T ;  // IMU到左相机变换
            else
                fs << "body_T_cam1" << cv_T ;  // IMU到右相机变换
        }
        // 释放文件存储器
        fs.release();
    }

    // 统计处理时间（用于性能分析）
    static double sum_of_time = 0;       // 累计总时间
    static int sum_of_calculation = 0;   // 累计处理帧数
    
    sum_of_time += t;
    sum_of_calculation++;
    
    // 输出当前帧处理时间和平均处理时间
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    // 计算并统计路径总长度
    // 计算当前帧位置与上一帧位置的欧氏距离
    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];  // 更新上一帧位置
    ROS_DEBUG("sum of path %f", sum_of_path);
    
    // 如果估计了时间延迟，输出时间延迟值
    if (ESTIMATE_TD)
        ROS_INFO("td %f", estimator.td);
}

// ==================== 发布里程计信息 ====================
// 功能：发布优化后的完整里程计信息，包括位置、姿态、速度，并记录到文件和路径中
// 参数：
//   estimator: 估计器对象，包含所有状态估计
//   header: ROS消息头（时间戳和坐标系信息）
void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    // 只有在非线性优化阶段才发布里程计（初始化完成后）
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        // 1. 创建里程计消息
        nav_msgs::Odometry odometry;
        odometry.header = header;                    // 使用传入的消息头
        odometry.header.frame_id = "world";          // 父坐标系：世界坐标系
        odometry.child_frame_id = "world";           // 子坐标系：这里设为world，通常应为body或imu
        
        // 2. 获取当前帧的旋转矩阵并转换为四元数
        // WINDOW_SIZE是滑动窗口大小，WINDOW_SIZE位置存储的是最新帧
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);  // 旋转矩阵转四元数
        
        // 3. 设置位置
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        
        // 4. 设置姿态（四元数）
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        
        // 5. 设置速度
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        
        // 6. 发布里程计消息
        pub_odometry.publish(odometry);

        // 7. 更新并发布路径（用于轨迹可视化）
        // 创建PoseStamped消息
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;  // 使用相同的位姿
        
        // 更新全局路径对象
        path.header = header;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);  // 将新位姿添加到路径末尾
        
        // 发布路径
        pub_path.publish(path);

        // 8. 将结果写入文件（用于后续评估和离线分析）
        // VINS_RESULT_PATH是预定义的结果文件路径
        ofstream foutC(VINS_RESULT_PATH, ios::app);  // 以追加模式打开文件
        
        // 设置输出格式：定点表示法，精度控制
        foutC.setf(ios::fixed, ios::floatfield);  // 固定小数点表示
        foutC.precision(0);                       // 时间戳用整数（纳秒）
        foutC << header.stamp.toSec() * 1e9 << ",";  // 写入时间戳（纳秒）
        
        foutC.precision(5);  // 位置和姿态用5位小数
        // 写入位置(x,y,z)
        foutC << estimator.Ps[WINDOW_SIZE].x() << ","
              << estimator.Ps[WINDOW_SIZE].y() << ","
              << estimator.Ps[WINDOW_SIZE].z() << ",";
        
        // 写入四元数(w,x,y,z)
        foutC << tmp_Q.w() << ","
              << tmp_Q.x() << ","
              << tmp_Q.y() << ","
              << tmp_Q.z() << ",";
        
        // 写入速度(vx,vy,vz)
        foutC << estimator.Vs[WINDOW_SIZE].x() << ","
              << estimator.Vs[WINDOW_SIZE].y() << ","
              << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
        
        foutC.close();  // 关闭文件

        // 9. 控制台输出调试信息
        Eigen::Vector3d tmp_T = estimator.Ps[WINDOW_SIZE];
        printf("time: %f, t: %f %f %f q: %f %f %f %f \n", 
               header.stamp.toSec(),           // 时间戳（秒）
               tmp_T.x(), tmp_T.y(), tmp_T.z(), // 位置
               tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z()); // 四元数
    }
}

// ==================== 发布关键帧位姿 ====================
// 功能：以可视化标记（点云）的形式发布关键帧位姿
// 参数：
//   estimator: 估计器对象
//   header: ROS消息头
void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
    // 如果没有关键帧，直接返回
    if (estimator.key_poses.size() == 0)
        return;
    
    // 1. 创建可视化标记（Marker）
    visualization_msgs::Marker key_poses;
    
    // 2. 设置Marker的基本属性
    key_poses.header = header;                  // 消息头
    key_poses.header.frame_id = "world";        // 坐标系：世界
    key_poses.ns = "key_poses";                 // 命名空间（用于RViz过滤）
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST; // 球体列表
    key_poses.action = visualization_msgs::Marker::ADD;      // 添加动作
    key_poses.pose.orientation.w = 1.0;         // 方向设为单位四元数
    key_poses.lifetime = ros::Duration();       // 生存时间（0表示永远）
    
    // 3. 设置Marker的ID和外观
    // static int key_poses_id = 0; // 注释掉的代码：使用静态ID
    key_poses.id = 0; // key_poses_id++; // 固定ID为0，如果多个关键帧标记需要不同ID
    key_poses.scale.x = 0.05;  // 球体X方向大小
    key_poses.scale.y = 0.05;  // 球体Y方向大小
    key_poses.scale.z = 0.05;  // 球体Z方向大小
    key_poses.color.r = 1.0;   // 颜色：红色
    key_poses.color.a = 1.0;   // 透明度：不透明
    
    // 4. 遍历滑动窗口中的所有帧（包括最新帧）
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        
        // 获取第i帧的位姿（已经校正过）
        correct_pose = estimator.key_poses[i];
        
        // 将Eigen向量转换为ROS Point
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        
        // 添加到Marker的点列表中
        key_poses.points.push_back(pose_marker);
    }
    
    // 5. 发布关键帧位姿标记
    pub_key_poses.publish(key_poses);
}


// ==================== 发布相机位姿 ====================
// 功能：发布相机坐标系下的位姿（与IMU位姿不同，需要考虑相机-IMU外参）
// 参数：
//   estimator: 估计器对象
//   header: ROS消息头
void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header)
{
    // 使用滑动窗口中倒数第二帧（WINDOW_SIZE-1）的相机位姿
    // 注意：WINDOW_SIZE是窗口大小，WINDOW_SIZE位置是最新帧
    int idx2 = WINDOW_SIZE - 1;

    // 只在非线性优化阶段发布（初始化完成后）
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;  // 使用倒数第二帧索引
        
        // 计算左相机在世界坐标系中的位置
        // 公式：P_cam = P_imu + R_imu * t_ic
        // 其中：P_imu是IMU位置，R_imu是IMU旋转，t_ic是IMU到相机的平移外参
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        
        // 计算左相机在世界坐标系中的旋转
        // 公式：R_cam = R_imu * R_ic
        // 其中：R_ic是IMU到相机的旋转外参
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        // 1. 创建并发布相机位姿的Odometry消息
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        
        // 设置相机位置
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        
        // 设置相机姿态
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        // 发布相机位姿
        pub_camera_pose.publish(odometry);

        // 2. 使用CameraPoseVisualization可视化相机模型
        cameraposevisual.reset();  // 重置之前的可视化
        
        // 添加左相机位姿
        cameraposevisual.add_pose(P, R);
        
        // 如果是双目系统，添加右相机位姿
        if(STEREO)
        {
            // 计算右相机在世界坐标系中的位置和旋转
            Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[1];
            Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[1]);
            cameraposevisual.add_pose(P, R);
        }
        
        // 发布相机位姿可视化（MarkerArray）
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}

// ==================== 发布点云 ====================
// 功能：发布特征点云地图，包括当前有效点云和边缘化点云
// 参数：
//   estimator: 估计器对象
//   header: ROS消息头
void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header)
{
    // 创建两个点云消息：正常点云和回环点云（回环点云这里没有使用）
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;

    // 1. 发布正常特征点云（当前活跃的特征点）
    // 遍历特征管理器中的所有特征
    for (auto &it_per_id : estimator.f_manager.feature)
    {
        // 获取该特征被观测的次数
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        
        // 条件1：特征至少被2帧观测到，且起始帧在滑动窗口的较老位置
        // WINDOW_SIZE - 2 确保特征不是最近两帧才出现的（为了稳定性）
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        
        // 条件2：特征起始帧不能太新（窗口的3/4之后），且三角化成功（solve_flag=1）
        // 排除太新的特征，因为它们可能不够稳定
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        
        // 计算特征点的世界坐标
        int imu_i = it_per_id.start_frame;  // 特征起始帧
        
        // 将特征在起始帧相机坐标系下的坐标转换为世界坐标：
        // 1. pts_i: 在起始帧相机坐标系下的3D坐标 = 归一化坐标 * 估计深度
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        
        // 2. w_pts_i: 转换到世界坐标系
        //    ric[0]*pts_i + tic[0]: 从相机坐标系到IMU坐标系
        //    Rs[imu_i] * (...): 从IMU坐标系旋转到世界坐标系
        //    + Ps[imu_i]: 加上IMU在世界坐标系中的位置
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) 
                         + estimator.Ps[imu_i];

        // 将Eigen向量转换为geometry_msgs::Point32
        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        
        // 添加到点云
        point_cloud.points.push_back(p);
    }
    
    // 发布正常点云
    pub_point_cloud.publish(point_cloud);

    // 2. 发布边缘化点云（将被移除的特征点）
    sensor_msgs::PointCloud margin_cloud;
    margin_cloud.header = header;

    // 再次遍历所有特征，筛选出边缘化特征
    for (auto &it_per_id : estimator.f_manager.feature)
    { 
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        
        // 同样需要满足基本条件：至少被2帧观测到，且起始帧较老
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        
        // 注释掉的条件：之前也考虑特征起始帧和三角化状态
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        // 边缘化点的条件：起始帧为0（最老帧），且只被不超过2帧观测到，但三角化成功
        // 这些特征即将被边缘化（移除出滑动窗口）
        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 
            && it_per_id.solve_flag == 1 )
        {
            // 同样计算世界坐标
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) 
                             + estimator.Ps[imu_i];

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            
            // 添加到边缘化点云
            margin_cloud.points.push_back(p);
        }
    }
    
    // 发布边缘化点云
    pub_margin_cloud.publish(margin_cloud);
}

// ==================== 发布TF变换 ====================
// 功能：发布世界坐标系到IMU(body)坐标系、IMU到相机坐标系的TF变换
// 作用：在RViz中可以通过TF树显示坐标系关系，便于可视化调试
void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
    // 只在非线性优化阶段发布TF（初始化完成后）
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    
    // 创建TF广播器（静态变量，只需创建一次）
    static tf::TransformBroadcaster br;
    
    // =========== 1. 发布世界坐标系到IMU(body)坐标系的变换 ===========
    tf::Transform transform;
    tf::Quaternion q;
    
    // body frame（IMU坐标系）
    Vector3d correct_t;      // 位置
    Quaterniond correct_q;   // 旋转
    
    // 获取最新帧（WINDOW_SIZE）的IMU位姿
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    // 设置变换的平移部分（世界坐标系到IMU坐标系）
    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    
    // 设置变换的旋转部分（四元数）
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    
    // 发布TF变换：从世界坐标系（父坐标系）到body坐标系（子坐标系）
    // 时间戳使用header.stamp，确保与其他消息同步
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    // =========== 2. 发布IMU(body)坐标系到相机坐标系的变换 ===========
    // 这是相机-IMU外参变换（通常固定不变）
    
    // 设置平移：tic[0]是左相机到IMU的平移
    transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    
    // 设置旋转：ric[0]是左相机到IMU的旋转矩阵，转换为四元数
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);
    
    // 发布TF变换：从body坐标系（父坐标系）到camera坐标系（子坐标系）
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

    // =========== 3. 发布外参消息（备用） ===========
    // 以Odometry消息形式发布外参，方便其他节点使用
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    
    // 设置外参平移
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    
    // 设置外参旋转
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    
    // 发布外参消息
    pub_extrinsic.publish(odometry);
}

// ==================== 发布关键帧 ====================
// 功能：发布关键帧的位姿和关联的地图点，用于建图、回环检测等
// 注意：只有在边缘化旧帧（MARGIN_OLD）时才发布关键帧
void pubKeyframe(const Estimator &estimator)
{
    // pub camera pose, 2D-3D points of keyframe
    // 条件：系统处于非线性优化阶段，且边缘化策略是边缘化最老帧
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        // 选择倒数第二帧作为关键帧（WINDOW_SIZE-2）
        int i = WINDOW_SIZE - 2;
        
        // 获取关键帧的IMU位姿（注意：这里用的是IMU位姿，不是相机位姿）
        // 注释掉的代码：使用相机位姿，实际使用IMU位姿
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];  // IMU位置
        Quaterniond R = Quaterniond(estimator.Rs[i]);  // IMU旋转

        // 1. 创建并发布关键帧位姿消息
        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);  // 关键帧时间戳
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        // 调试输出
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose.publish(odometry);

        // 2. 创建并发布关键帧关联的点云（包含3D点和对应的2D观测）
        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
        point_cloud.header.frame_id = "world";
        
        // 遍历所有特征点
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();  // 特征被观测的帧数
            
            // 条件：特征起始帧早于关键帧，且特征在当前关键帧（WINDOW_SIZE-2）中被观测到
            // 并且特征三角化成功（solve_flag == 1）
            if(it_per_id.start_frame < WINDOW_SIZE - 2 && 
               it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && 
               it_per_id.solve_flag == 1)
            {
                // 计算特征点的世界坐标（与pubPointCloud中相同）
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                                      + estimator.Ps[imu_i];
                
                // 添加3D点
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                // 添加对应的2D观测信息（存储在channels中）
                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;  // 特征在关键帧中的观测索引
                sensor_msgs::ChannelFloat32 p_2d;
                
                // values[0,1]: 归一化平面坐标 (x, y)
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                
                // values[2,3]: 像素坐标 (u, v)（可能用于可视化）
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                
                // values[4]: 特征ID
                p_2d.values.push_back(it_per_id.feature_id);
                
                // 添加到channels（每个点对应一个channel）
                point_cloud.channels.push_back(p_2d);
            }
        }
        
        // 发布关键帧点云
        pub_keyframe_point.publish(point_cloud);
    }
}