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
#include <vector>
#include <eigen3/Eigen/Dense>
#include "../utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <map>

using namespace std;

// VINS-Mono/VINS-Fusion 视觉惯性SLAM系统参数配置文件注释

// 相机焦距（像素单位），用于特征点投影和三角化
const double FOCAL_LENGTH = 460.0;

// 滑动窗口的大小，决定优化时保留多少帧数据
const int WINDOW_SIZE = 10;

// 最大特征点数量，控制每帧图像提取的特征点上限
const int NUM_OF_F = 1000;

// 定义是否使用单位球面误差模型（注释掉的宏定义）
// 如果启用，重投影误差将在单位球面上计算，适用于鱼眼相机
//#define UNIT_SPHERE_ERROR

// 外部变量声明 - 这些变量在别的源文件中定义

// 初始深度值，用于初始化特征点的深度
extern double INIT_DEPTH;

// 最小视差阈值，用于判断是否添加新的关键帧
// 当前帧与上一关键帧的视差小于此值时，可能不添加为关键帧
extern double MIN_PARALLAX;

// 外参标定模式：
// 0: 使用固定外参
// 1: 只估计旋转外参
// 2: 估计完整的旋转和平移外参
extern int ESTIMATE_EXTRINSIC;

// IMU噪声参数 - 这些是IMU的固有特性参数
extern double ACC_N, ACC_W;  // 加速度计测量噪声和随机游走噪声
extern double GYR_N, GYR_W;  // 陀螺仪测量噪声和随机游走噪声

// 相机与IMU之间的外参（旋转和平移）
// RIC: 相机到IMU的旋转矩阵
// TIC: 相机到IMU的平移向量
extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;

// 重力向量，在初始对齐中估计
extern Eigen::Vector3d G;

// IMU偏置阈值，用于检测和重置过大的偏置值
extern double BIAS_ACC_THRESHOLD;  // 加速度计偏置阈值
extern double BIAS_GYR_THRESHOLD;  // 陀螺仪偏置阈值

// 非线性优化求解器参数
extern double SOLVER_TIME;        // 最大求解时间（秒）
extern int NUM_ITERATIONS;        // 优化迭代次数

// 输出文件路径
extern std::string EX_CALIB_RESULT_PATH;  // 外参标定结果保存路径
extern std::string VINS_RESULT_PATH;      // VINS轨迹结果保存路径
extern std::string OUTPUT_FOLDER;         // 输出文件夹路径

// ROS话题名称
extern std::string IMU_TOPIC;     // IMU数据话题名

// 时间戳相关参数
extern double TD;                 // 相机和IMU之间的时间偏移（时间标定）
extern int ESTIMATE_TD;          // 是否估计时间偏移：0-不估计，1-估计

// 相机参数
extern int ROLLING_SHUTTER;      // 是否为卷帘快门相机：0-全局快门，1-卷帘快门
extern int ROW, COL;             // 图像的行数和列数（分辨率）
extern int NUM_OF_CAM;           // 相机数量（1-单目，2-双目等）
extern int STEREO;               // 是否为双目系统：0-单目，1-双目

// 系统功能开关
extern int USE_IMU;              // 是否使用IMU数据：0-纯视觉，1-视觉惯性融合
extern int MULTIPLE_THREAD;      // 是否使用多线程：0-单线程，1-多线程

// 用于调试的地面真值点（将估计的特征点与真实值比较）
extern map<int, Eigen::Vector3d> pts_gt;

// 图像话题名称（双目系统有两个图像话题）
extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC;

// 鱼眼相机掩模文件路径，用于去除鱼眼图像的边缘畸变区域
extern std::string FISHEYE_MASK;

// 相机配置文件路径列表
extern std::vector<std::string> CAM_NAMES;

// 特征点跟踪参数
extern int MAX_CNT;              // 每帧图像最大特征点数量
extern int MIN_DIST;             // 特征点之间的最小像素距离（避免特征点聚集）
extern double F_THRESHOLD;       // 基础矩阵/F矩阵的RANSAC阈值
extern int SHOW_TRACK;           // 是否可视化特征点跟踪：0-不显示，1-显示
extern int FLOW_BACK;            // 是否进行前向后向光流一致性检查：0-否，1-是

// 参数读取函数声明，从配置文件中读取所有参数
void readParameters(std::string config_file);

// 状态参数化维度枚举
enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,      // 姿态维度：4(四元数) + 3(位置) = 7
    SIZE_SPEEDBIAS = 9, // 速度与偏置维度：3(速度) + 3(加速度偏置) + 3(陀螺仪偏置) = 9
    SIZE_FEATURE = 1    // 特征点维度：逆深度为1维
};

// 状态向量中的顺序枚举
enum StateOrder
{
    O_P = 0,   // 位置起始索引
    O_R = 3,   // 旋转（四元数）起始索引
    O_V = 6,   // 速度起始索引
    O_BA = 9,  // 加速度计偏置起始索引
    O_BG = 12  // 陀螺仪偏置起始索引
};

// 噪声向量中的顺序枚举
enum NoiseOrder
{
    O_AN = 0,  // 加速度计测量噪声起始索引
    O_GN = 3,  // 陀螺仪测量噪声起始索引
    O_AW = 6,  // 加速度计随机游走噪声起始索引
    O_GW = 9   // 陀螺仪随机游走噪声起始索引
};