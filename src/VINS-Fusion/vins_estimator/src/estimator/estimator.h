/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
 
#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../featureTracker/feature_tracker.h"


// Estimator类：VIO系统的主要状态估计器，负责融合IMU和视觉数据
class Estimator
{
  public:
    // 构造函数和析构函数
    Estimator();
    ~Estimator();
    
    // 设置系统参数（从配置文件中读取的参数）
    void setParameter();

    // =========== 对外接口函数 ===========
    
    // 初始化第一帧位姿（通常从外部输入，如GPS或手动设置）
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
    
    // 输入IMU数据（时间戳、线加速度、角速度）
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
    
    // 输入特征点数据（时间戳、特征点观测）
    void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);
    
    // 输入图像数据（时间戳、左图像、可选的右图像）
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    
    // 处理IMU数据（预积分）
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    
    // 处理图像特征数据
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header);
    
    // 处理所有测量数据的主循环
    void processMeasurements();
    
    // 改变传感器配置（是否使用IMU、双目）
    void changeSensorType(int use_imu, int use_stereo);

    // =========== 内部处理函数 ===========
    
    // 清空所有状态
    void clearState();
    
    // 初始化：通过SFM（Structure from Motion）估计初始结构和运动
    bool initialStructure();
    
    // 视觉惯性对齐：将视觉SFM结果与IMU预积分对齐
    bool visualInitialAlign();
    
    // 计算相对位姿（用于初始化）
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    
    // 滑动窗口操作（主函数）
    void slideWindow();
    
    // 滑动窗口（当新帧是关键帧时）
    void slideWindowNew();
    
    // 滑动窗口（边缘化最老帧时）
    void slideWindowOld();
    
    // 非线性优化（Bundle Adjustment + IMU）
    void optimization();
    
    // 将状态变量从向量形式转换到double数组（用于Ceres优化）
    void vector2double();
    
    // 将double数组转换回状态变量向量
    void double2vector();
    
    // 失败检测：判断系统是否跟踪失败
    bool failureDetection();
    
    // 获取指定时间间隔内的IMU数据
    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                              vector<pair<double, Eigen::Vector3d>> &gyrVector);
    
    // 获取世界坐标系下的当前位姿（4x4变换矩阵）
    void getPoseInWorldFrame(Eigen::Matrix4d &T);
    
    // 获取世界坐标系下指定帧的位姿
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
    
    // 预测特征点在下一帧的位置（用于光流跟踪）
    void predictPtsInNextFrame();
    
    // 异常值剔除（基于重投影误差）
    void outliersRejection(set<int> &removeIndex);
    
    // 计算重投影误差
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                     Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                     double depth, Vector3d &uvi, Vector3d &uvj);
    
    // 更新最新状态（用于输出）
    void updateLatestStates();
    
    // 快速IMU预测（用于两帧之间的状态预测）
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    
    // 检查是否有可用的IMU数据
    bool IMUAvailable(double t);
    
    // 利用初始IMU数据估计重力方向
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);

    // =========== 枚举类型定义 ===========
    
    // 求解器状态标志
    enum SolverFlag
    {
        INITIAL,    // 初始化阶段
        NON_LINEAR  // 非线性优化阶段
    };

    // 边缘化策略标志
    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,       // 边缘化最老的帧（标准滑动窗口）
        MARGIN_SECOND_NEW = 1 // 边缘化次新的帧（当新帧不是关键帧时）
    };

    // =========== 多线程同步相关 ===========
    
    std::mutex mProcess;     // 处理过程锁
    std::mutex mBuf;         // 缓冲区锁
    std::mutex mPropagate;   // 传播锁（用于IMU预测）
    
    // =========== 数据缓冲区 ===========
    
    queue<pair<double, Eigen::Vector3d>> accBuf;  // 加速度计缓冲区
    queue<pair<double, Eigen::Vector3d>> gyrBuf;  // 陀螺仪缓冲区
    // 特征点缓冲区：时间戳 + 特征点数据（ID -> (相机ID, 7维观测)）
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;
    
    // =========== 时间相关 ===========
    
    double prevTime, curTime;  // 前一时间和当前时间
    bool openExEstimation;     // 是否在线估计外参

    // =========== 线程 ===========
    
    std::thread trackThread;    // 特征跟踪线程
    std::thread processThread;  // 处理线程

    // =========== 特征跟踪器 ===========
    
    FeatureTracker featureTracker;  // 特征跟踪器实例

    // =========== 系统状态标志 ===========
    
    SolverFlag solver_flag;                 // 求解器状态
    MarginalizationFlag marginalization_flag; // 边缘化策略
    Vector3d g;                             // 重力向量

    // =========== 外参参数 ===========
    
    Matrix3d ric[2];  // 相机到IMU的旋转外参（0:左目，1:右目）
    Vector3d tic[2];  // 相机到IMU的平移外参

    // =========== 滑动窗口状态变量 ===========
    // 窗口大小为WINDOW_SIZE，额外+1用于存储最新帧
    
    Vector3d        Ps[(WINDOW_SIZE + 1)];  // 位置（世界坐标系到IMU坐标系）
    Vector3d        Vs[(WINDOW_SIZE + 1)];  // 速度（世界坐标系下）
    Matrix3d        Rs[(WINDOW_SIZE + 1)];  // 旋转（世界坐标系到IMU坐标系）
    Vector3d        Bas[(WINDOW_SIZE + 1)]; // 加速度计偏置
    Vector3d        Bgs[(WINDOW_SIZE + 1)]; // 陀螺仪偏置
    double td;                              // 时间戳偏移（相机和IMU之间的时间同步误差）

    // =========== 位姿相关缓存 ===========
    
    Matrix3d back_R0, last_R, last_R0;  // 用于边缘化的旋转缓存
    Vector3d back_P0, last_P, last_P0;  // 用于边缘化的平移缓存
    double Headers[(WINDOW_SIZE + 1)];  // 每帧的时间戳

    // =========== IMU预积分相关 ===========
    
    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];  // 预积分器数组
    Vector3d acc_0, gyr_0;  // 上一时刻的加速度和角速度

    // =========== IMU数据缓存 ===========
    
    vector<double> dt_buf[(WINDOW_SIZE + 1)];                       // 时间间隔缓存
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];    // 加速度缓存
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];       // 角速度缓存

    // =========== 帧计数和统计 ===========
    
    int frame_count;           // 当前帧在滑动窗口中的索引
    int sum_of_outlier;        // 异常值总数统计
    int sum_of_back;           // 后端优化次数统计
    int sum_of_front;          // 前端跟踪次数统计
    int sum_of_invalid;        // 无效特征统计
    int inputImageCnt;         // 输入图像计数

    // =========== 管理器对象 ===========
    
    FeatureManager f_manager;      // 特征管理器
    MotionEstimator m_estimator;   // 运动估计器（用于初始化）
    InitialEXRotation initial_ex_rotation;  // 初始外参旋转估计器

    // =========== 系统标志 ===========
    
    bool first_imu;        // 是否收到第一个IMU数据
    bool is_valid;         // 当前状态是否有效
    bool is_key;           // 当前帧是否为关键帧
    bool failure_occur;    // 是否发生跟踪失败

    // =========== 点云和位姿可视化数据 ===========
    
    vector<Vector3d> point_cloud;   // 点云数据（用于可视化）
    vector<Vector3d> margin_cloud;  // 边缘化的点云
    vector<Vector3d> key_poses;     // 关键帧位姿
    double initial_timestamp;       // 初始时间戳

    // =========== 优化变量（Ceres使用） ===========
    
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];          // 位姿参数（旋转+平移）
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];// 速度+偏置参数
    double para_Feature[NUM_OF_F][SIZE_FEATURE];           // 特征点参数（逆深度）
    double para_Ex_Pose[2][SIZE_POSE];                     // 外参参数
    double para_Retrive_Pose[SIZE_POSE];                   // 检索位姿（用于重定位）
    double para_Td[1][1];                                  // 时间延迟参数
    double para_Tr[1][1];                                  // 其他变换参数（如轮速计）

    // =========== 回环检测相关 ===========
    
    int loop_window_index;  // 回环检测窗口索引

    // =========== 边缘化相关 ===========
    
    MarginalizationInfo *last_marginalization_info;  // 上一次边缘化的信息
    vector<double *> last_marginalization_parameter_blocks;  // 边缘化的参数块

    // =========== 初始化相关 ===========
    
    map<double, ImageFrame> all_image_frame;  // 所有图像帧（用于初始化）
    IntegrationBase *tmp_pre_integration;     // 临时预积分器

    // =========== 初始位姿 ===========
    
    Eigen::Vector3d initP;  // 初始位置
    Eigen::Matrix3d initR;  // 初始旋转

    // =========== 最新状态（用于输出） ===========
    
    double latest_time;           // 最新时间戳
    Eigen::Vector3d latest_P;     // 最新位置
    Eigen::Vector3d latest_V;     // 最新速度
    Eigen::Vector3d latest_Ba;    // 最新加速度计偏置
    Eigen::Vector3d latest_Bg;    // 最新陀螺仪偏置
    Eigen::Vector3d latest_acc_0; // 最新加速度
    Eigen::Vector3d latest_gyr_0; // 最新角速度
    Eigen::Quaterniond latest_Q;  // 最新四元数

    // =========== 初始化标志 ===========
    
    bool initFirstPoseFlag;  // 第一帧位姿是否已初始化
    bool initThreadFlag;     // 初始化线程是否已启动
};