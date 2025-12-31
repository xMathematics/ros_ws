/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator/estimator.h"
#include "../estimator/parameters.h"
#include <fstream>

// ==================== 全局发布器声明 ====================
// 这些是在其他文件中定义的全局ROS发布器，用于发布不同类型的消息

// 发布里程计消息（包含位置、姿态、速度等完整状态信息）
extern ros::Publisher pub_odometry;

// 发布路径消息和位姿消息（路径用于可视化轨迹，位姿用于单帧位姿）
extern ros::Publisher pub_path, pub_pose;

// 发布点云地图和局部点云（pub_cloud可能用于当前帧点云，pub_map用于全局地图）
extern ros::Publisher pub_cloud, pub_map;

// 发布关键帧位姿（用于显示关键帧轨迹）
extern ros::Publisher pub_key_poses;

// 发布参考帧和当前帧位姿（用于对比显示）
extern ros::Publisher pub_ref_pose, pub_cur_pose;

// 发布关键帧（可能包含关键帧的完整信息）
extern ros::Publisher pub_key;

// 全局路径对象，用于存储和发布历史位姿构成的路径
extern nav_msgs::Path path;

// 发布位姿图（用于回环检测和全局优化）
extern ros::Publisher pub_pose_graph;

// 图像尺寸参数（从配置文件读取）
extern int IMAGE_ROW, IMAGE_COL;

// ==================== 初始化函数 ====================
// 注册所有发布器到ROS节点
// 参数：n - ROS节点句柄
void registerPub(ros::NodeHandle &n);

// ==================== 实时数据发布函数 ====================

// 发布最新的里程计信息（高频，通常用于控制或实时显示）
// 参数：
//   P: 位置 (x, y, z)
//   Q: 旋转（四元数 w, x, y, z）
//   V: 速度 (vx, vy, vz)
//   t: 时间戳
void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, 
                       const Eigen::Vector3d &V, double t);

// 发布跟踪图像（将特征点跟踪结果可视化在图像上）
// 参数：
//   imgTrack: 带有特征点跟踪标记的图像
//   t: 时间戳
void pubTrackImage(const cv::Mat &imgTrack, const double t);

// 打印统计信息（用于调试和性能监控）
// 参数：
//   estimator: 估计器对象，包含各种统计信息
//   t: 时间戳
void printStatistics(const Estimator &estimator, double t);

// ==================== 估计结果发布函数 ====================

// 发布完整的里程计信息（优化后的结果）
// 参数：
//   estimator: 估计器对象，包含所有状态估计
//   header: ROS消息头（时间戳和坐标系）
void pubOdometry(const Estimator &estimator, const std_msgs::Header &header);

// 发布初始猜测位姿（用于对比优化前后的结果）
void pubInitialGuess(const Estimator &estimator, const std_msgs::Header &header);

// 发布关键帧位姿（从滑动窗口中提取关键帧）
void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header);

// 发布相机位姿（转换为相机坐标系的可视化）
void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header);

// 发布点云地图（当前观测到的或全局的点云）
void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header);

// 发布TF变换（用于RViz中的坐标系显示）
void pubTF(const Estimator &estimator, const std_msgs::Header &header);

// 发布关键帧（包含关键帧的完整信息，如位姿、特征点等）
void pubKeyframe(const Estimator &estimator);

// 发布重定位结果（用于回环检测或重定位时）
void pubRelocalization(const Estimator &estimator);

// 发布车辆模型（如果有的话，用于可视化车辆形状）
void pubCar(const Estimator & estimator, const std_msgs::Header &header);
