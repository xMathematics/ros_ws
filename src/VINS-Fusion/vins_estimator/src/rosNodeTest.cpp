/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;

// IMU数据缓冲区队列，存储接收到的IMU消息指针
queue<sensor_msgs::ImuConstPtr> imu_buf;

// 特征点数据缓冲区队列，存储接收到的特征点云消息指针
queue<sensor_msgs::PointCloudConstPtr> feature_buf;

// 左目图像数据缓冲区队列（相机0），存储接收到的图像消息指针
queue<sensor_msgs::ImageConstPtr> img0_buf;

// 右目图像数据缓冲区队列（相机1），存储接收到的图像消息指针
queue<sensor_msgs::ImageConstPtr> img1_buf;

// 互斥锁，用于保护对缓冲区的并发访问，防止数据竞争
std::mutex m_buf;

// 左目相机的图像消息回调函数
// 参数：img_msg - 接收到的图像消息的智能指针（常量引用）
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // 获取互斥锁，确保对缓冲区的访问是线程安全的
    m_buf.lock();
    
    // 将接收到的图像消息指针压入左目图像缓冲区队列
    img0_buf.push(img_msg);
    
    // 释放互斥锁，允许其他线程访问缓冲区
    m_buf.unlock();
}

// 右目相机的图像消息回调函数
// 参数：img_msg - 接收到的图像消息的智能指针（常量引用）
void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // 获取互斥锁，确保对缓冲区的访问是线程安全的
    m_buf.lock();
    
    // 将接收到的图像消息指针压入右目图像缓冲区队列
    img1_buf.push(img_msg);
    
    // 释放互斥锁，允许其他线程访问缓冲区
    m_buf.unlock();
}


// 将ROS图像消息转换为OpenCV的Mat格式
// 参数：img_msg - ROS图像消息的智能指针
// 返回值：cv::Mat - OpenCV格式的图像数据
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    // 定义cv_bridge指针，用于图像格式转换
    cv_bridge::CvImageConstPtr ptr;
    
    // 检查图像编码格式是否为"8UC1"（8位无符号单通道）
    if (img_msg->encoding == "8UC1")
    {
        // 创建一个新的图像消息对象
        sensor_msgs::Image img;
        
        // 复制原始消息的头部信息
        img.header = img_msg->header;
        
        // 复制图像高度
        img.height = img_msg->height;
        
        // 复制图像宽度
        img.width = img_msg->width;
        
        // 复制字节序信息
        img.is_bigendian = img_msg->is_bigendian;
        
        // 复制步长（一行像素占用的字节数）
        img.step = img_msg->step;
        
        // 直接引用原始图像数据，避免不必要的拷贝
        img.data = img_msg->data;
        
        // 将编码格式更改为"mono8"（单通道8位灰度图像）
        img.encoding = "mono8";
        
        // 使用cv_bridge将ROS消息转换为OpenCV图像，指定输出格式为MONO8
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else  // 如果编码格式不是"8UC1"
    {
        // 直接使用原始消息，转换为MONO8格式的OpenCV图像
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }

    // 将cv_bridge转换的图像进行深拷贝，确保返回的图像数据是独立的
    cv::Mat img = ptr->image.clone();
    
    // 返回OpenCV格式的图像
    return img;
}

// extract images with same timestamp from two topics
// 从两个话题中提取时间戳相同的图像
void sync_process()
{
    while(1)  // 无限循环，持续处理图像
    {
        if(STEREO)  // 如果启用双目模式
        {
            cv::Mat image0, image1;  // 存储左右目图像的Mat对象
            std_msgs::Header header;  // 图像消息头
            double time = 0;  // 图像时间戳
            
            m_buf.lock();  // 加锁，保护共享缓冲区
            
            // 检查左右目图像缓冲区是否都不为空
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                // 获取左右目图像的时间戳
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                
                // 0.003s sync tolerance - 允许0.003秒的时间同步容差
                
                // 情况1：左目图像时间戳比右目早0.003秒以上
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();  // 丢弃左目图像（太旧）
                    printf("throw img0\n");
                }
                // 情况2：左目图像时间戳比右目晚0.003秒以上
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();  // 丢弃右目图像（太旧）
                    printf("throw img1\n");
                }
                // 情况3：时间戳在容差范围内，可以配对
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();  // 使用左目时间戳
                    header = img0_buf.front()->header;  // 获取消息头
                    
                    // 从消息中提取图像数据
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();  // 弹出已处理的左目图像
                    
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();  // 弹出已处理的右目图像
                    
                    //printf("find img0 and img1\n");  // 可选的调试信息
                }
            }
            m_buf.unlock();  // 解锁缓冲区
            
            // 如果成功提取到图像，则输入到估计器
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else  // 单目模式
        {
            cv::Mat image;  // 存储单目图像
            std_msgs::Header header;
            double time = 0;
            
            m_buf.lock();  // 加锁
            if(!img0_buf.empty())  // 只要左目缓冲区有图像
            {
                time = img0_buf.front()->header.stamp.toSec();  // 获取时间戳
                header = img0_buf.front()->header;  // 获取消息头
                image = getImageFromMsg(img0_buf.front());  // 提取图像
                img0_buf.pop();  // 弹出已处理的图像
            }
            m_buf.unlock();  // 解锁
            
            // 如果成功提取到图像，则输入到估计器（单目版本）
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        // 线程休眠2毫秒，避免过度占用CPU
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


// IMU消息回调函数
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // 获取IMU消息的时间戳并转换为秒
    double t = imu_msg->header.stamp.toSec();
    
    // 提取线性加速度的三个分量
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    
    // 提取角速度的三个分量
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    
    // 创建3D向量存储加速度和角速度
    Vector3d acc(dx, dy, dz);   // 加速度向量 (m/s²)
    Vector3d gyr(rx, ry, rz);   // 角速度向量 (rad/s)
    
    // 将IMU数据输入到估计器中进行处理
    estimator.inputIMU(t, acc, gyr);
    
    return;  // 函数返回
}

// 特征点云消息回调函数
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    // 定义特征帧数据结构：特征ID -> [相机ID, 特征数据]的列表
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    // 特征数据格式 (7维向量)：
    // [0]: x - 归一化平面x坐标 (z=1)
    // [1]: y - 归一化平面y坐标 (z=1)  
    // [2]: z - 归一化平面z坐标 (固定为1)
    // [3]: p_u - 像素u坐标
    // [4]: p_v - 像素v坐标
    // [5]: velocity_x - 像素速度x分量
    // [6]: velocity_y - 像素速度y分量
    
    // 遍历特征消息中的所有特征点
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        // 从channels[0]获取特征点的唯一ID
        int feature_id = feature_msg->channels[0].values[i];
        
        // 从channels[1]获取观察到此特征点的相机ID（0-左目，1-右目）
        int camera_id = feature_msg->channels[1].values[i];
        
        // 获取特征点在归一化平面上的坐标 (z=1)
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        
        // 从channels[2-3]获取特征点在图像上的像素坐标
        double p_u = feature_msg->channels[2].values[i];  // 像素u坐标
        double p_v = feature_msg->channels[3].values[i];  // 像素v坐标
        
        // 从channels[4-5]获取特征点在图像上的像素速度（光流）
        double velocity_x = feature_msg->channels[4].values[i];  // 像素速度x
        double velocity_y = feature_msg->channels[5].values[i];  // 像素速度y
        
        // 如果有地面真值信息（通道数大于5），则获取并存储
        if(feature_msg->channels.size() > 5)
        {
            // 获取特征点的3D地面真值坐标
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            
            // 将地面真值存储到全局map中（用于评估）
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            
            // 调试信息：打印接收到的地面真值点
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        
        // 断言：归一化平面上的z坐标必须为1（归一化点）
        ROS_ASSERT(z == 1);
        
        // 创建7维特征向量：包含归一化坐标、像素坐标和像素速度
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        // 向量元素顺序：x, y, z, u, v, vx, vy
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        
        // 将特征数据添加到特征帧中
        // 同一特征可能被多个相机观测到（双目），所以使用vector存储
        featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    }
    
    // 获取特征消息的时间戳
    double t = feature_msg->header.stamp.toSec();
    
    // 将特征帧数据输入到估计器中
    estimator.inputFeature(t, featureFrame);
    
    return;  // 函数返回
}

// 重启估计器回调函数
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    // 如果重启消息的数据为true
    if (restart_msg->data == true)
    {
        // 输出警告日志：重启估计器
        ROS_WARN("restart the estimator!");
        
        // 清空估计器的状态
        estimator.clearState();
        
        // 重新设置估计器的参数
        estimator.setParameter();
    }
    
    return;  // 函数返回
}

// IMU开关回调函数
void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    // 如果开关消息为true，启用IMU
    if (switch_msg->data == true)
    {
        // 可选的调试信息：ROS_WARN("use IMU!");
        
        // 更改传感器类型：启用IMU，STEREO参数保持不变
        estimator.changeSensorType(1, STEREO);
    }
    else  // 否则禁用IMU
    {
        // 可选的调试信息：ROS_WARN("disable IMU!");
        
        // 更改传感器类型：禁用IMU，STEREO参数保持不变
        estimator.changeSensorType(0, STEREO);
    }
    
    return;  // 函数返回
}

// 相机开关回调函数
void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    // 如果开关消息为true，启用双目模式
    if (switch_msg->data == true)
    {
        // 可选的调试信息：ROS_WARN("use stereo!");
        
        // 更改传感器类型：USE_IMU参数保持不变，启用双目（1）
        estimator.changeSensorType(USE_IMU, 1);
    }
    else  // 否则启用单目模式
    {
        // 可选的调试信息：ROS_WARN("use mono camera (left)!");
        
        // 更改传感器类型：USE_IMU参数保持不变，使用单目（0）
        estimator.changeSensorType(USE_IMU, 0);
    }
    
    return;  // 函数返回
}

// VIO系统的主函数入口
// 参数：
//   argc: 命令行参数数量
//   argv: 命令行参数数组
int main(int argc, char **argv)
{
    // 初始化ROS节点
    // "vins_estimator"是节点名称
    ros::init(argc, argv, "vins_estimator");
    
    // 创建节点句柄，使用私有命名空间（~）
    // 私有命名空间意味着参数可以相对节点名访问
    ros::NodeHandle n("~");
    
    // 设置ROS日志级别为Info
    // ROSCONSOLE_DEFAULT_NAME是默认日志记录器名称
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // 检查命令行参数数量
    // 程序需要1个参数：配置文件路径（argc=2，因为第一个参数是程序名）
    if(argc != 2)
    {
        // 输出使用说明
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;  // 返回非0表示程序异常结束
    }

    // 获取配置文件路径（第二个命令行参数）
    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    // 从配置文件中读取参数（全局函数，读取到全局变量中）
    readParameters(config_file);
    
    // 设置估计器的参数（将读取的参数应用到Estimator对象中）
    estimator.setParameter();

// 条件编译：如果定义了EIGEN_DONT_PARALLELIZE宏，则禁用Eigen并行计算
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    // 输出等待传感器数据的提示信息
    ROS_WARN("waiting for image and imu...");

    // 注册发布器（用于发布位姿、点云、路径等消息）
    registerPub(n);

    // 定义IMU订阅器
    ros::Subscriber sub_imu;
    
    // 如果系统配置使用IMU（USE_IMU是配置文件中的参数）
    if(USE_IMU)
    {
        // 订阅IMU话题
        // 参数：
        //   IMU_TOPIC: 话题名称（从配置文件读取）
        //   2000: 队列大小（缓存的消息数量）
        //   imu_callback: 回调函数，处理接收到的IMU数据
        //   ros::TransportHints().tcpNoDelay(): 传输提示，使用TCP无延迟模式（减少延迟）
        sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    }
    
    // 订阅特征点话题（来自特征跟踪节点）
    // 话题名："/feature_tracker/feature"
    // 队列大小：2000
    // 回调函数：feature_callback
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    
    // 订阅左目图像话题（用于特征跟踪，这里可能是备用或调试用途）
    // IMAGE0_TOPIC: 左目图像话题名称
    // 队列大小：100（图像数据量大，队列较小）
    // 回调函数：img0_callback
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    
    // 定义右目图像订阅器
    ros::Subscriber sub_img1;
    
    // 如果系统配置为双目（STEREO是配置文件中的参数）
    if(STEREO)
    {
        // 订阅右目图像话题
        sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    }
    
    // 订阅重启话题（用于系统重启）
    // 话题名："/vins_restart"
    // 回调函数：restart_callback
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    
    // 订阅IMU开关话题（用于动态启用/禁用IMU）
    // 话题名："/vins_imu_switch"
    // 回调函数：imu_switch_callback
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    
    // 订阅相机开关话题（用于动态启用/禁用相机）
    // 话题名："/vins_cam_switch"
    // 回调函数：cam_switch_callback
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    // 创建并启动同步处理线程
    // sync_process: 线程函数，负责同步和处理IMU和图像数据
    std::thread sync_thread{sync_process};
    
    // ROS主循环
    // 阻塞等待回调函数被调用，直到节点被关闭
    ros::spin();

    // 程序正常退出
    return 0;
}