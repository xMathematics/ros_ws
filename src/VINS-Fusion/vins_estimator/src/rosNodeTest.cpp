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

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;


void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
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