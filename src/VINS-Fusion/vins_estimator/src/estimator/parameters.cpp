/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

// VINS-Mono/VINS-Fusion 视觉惯性SLAM系统参数定义和初始化文件注释

// 定义并初始化外部变量（这些变量在头文件中声明为extern）

// 特征点的初始深度估计值（单位：米），用于特征点初始化
double INIT_DEPTH;

// 最小视差阈值（弧度），用于关键帧选择
double MIN_PARALLAX;

// IMU噪声参数定义
double ACC_N, ACC_W;  // 加速度计测量噪声和随机游走噪声标准差
double GYR_N, GYR_W;  // 陀螺仪测量噪声和随机游走噪声标准差

// 相机到IMU的外参变换容器（支持多个相机）
std::vector<Eigen::Matrix3d> RIC;  // 旋转矩阵：从相机坐标系到IMU坐标系
std::vector<Eigen::Vector3d> TIC;  // 平移向量：从相机坐标系到IMU坐标系

// 重力向量初始化（默认值为9.8m/s²，方向向下）
Eigen::Vector3d G{0.0, 0.0, 9.8};

// IMU偏置阈值，用于检测异常偏置
double BIAS_ACC_THRESHOLD;  // 加速度计偏置阈值（m/s²）
double BIAS_GYR_THRESHOLD;  // 陀螺仪偏置阈值（rad/s）

// 非线性优化求解器参数
double SOLVER_TIME;        // 最大求解时间限制（秒）
int NUM_ITERATIONS;        // 最大迭代次数

// 标定模式标志位
int ESTIMATE_EXTRINSIC;    // 外参估计模式
int ESTIMATE_TD;           // 时间偏移估计标志
int ROLLING_SHUTTER;       // 卷帘快门标志

// 文件路径字符串
std::string EX_CALIB_RESULT_PATH;  // 外参标定结果保存路径
std::string VINS_RESULT_PATH;      // VINS轨迹结果保存路径
std::string OUTPUT_FOLDER;         // 输出文件夹路径

// ROS话题名称
std::string IMU_TOPIC;     // IMU数据话题名

// 图像参数
int ROW, COL;              // 图像高度和宽度（像素）
double TD;                 // 相机和IMU之间的时间偏移（秒）

// 系统配置参数
int NUM_OF_CAM;            // 相机数量（1=单目，2=双目等）
int STEREO;                // 是否为双目系统（0=单目，1=双目）
int USE_IMU;               // 是否使用IMU数据（0=纯视觉，1=视觉惯性融合）
int MULTIPLE_THREAD;       // 是否使用多线程优化

// 地面真值点地图，用于调试（将估计的特征点与真实值比较）
map<int, Eigen::Vector3d> pts_gt;

// 图像话题名称
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;  // 左右相机图像话题名

// 鱼眼相机掩模文件路径
std::string FISHEYE_MASK;

// 相机内参配置文件路径列表
std::vector<std::string> CAM_NAMES;

// 特征跟踪参数
int MAX_CNT;               // 每帧最大特征点数量
int MIN_DIST;              // 特征点之间的最小像素距离
double F_THRESHOLD;        // 基础矩阵/F矩阵的RANSAC阈值
int SHOW_TRACK;            // 是否显示特征点跟踪可视化
int FLOW_BACK;             // 是否进行前向后向光流一致性检查

// 通用参数读取模板函数
// 功能：从ROS参数服务器读取指定名称的参数
// 参数：
//   n - ROS节点句柄
//   name - 参数名
// 返回值：读取到的参数值
// 异常处理：如果参数不存在，则关闭ROS节点
template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))  // 尝试从参数服务器获取参数
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();  // 参数加载失败，关闭节点
    }
    return ans;
}

// 从YAML配置文件读取所有参数的函数
// 参数：config_file - 配置文件路径
void readParameters(std::string config_file)
{
    // 检查配置文件是否存在
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();  // 停止程序执行
        return;          
    }
    fclose(fh);

    // 使用OpenCV的FileStorage类读取YAML配置文件
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())  // 检查文件是否成功打开
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    // 读取图像话题名称
    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    
    // 读取特征跟踪参数
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    // 读取多线程标志
    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    // 读取IMU使用标志
    USE_IMU = fsSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    
    // 如果使用IMU，读取IMU相关参数
    if(USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        
        // 读取IMU噪声参数
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        
        // 读取重力加速度值
        G.z() = fsSettings["g_norm"];
    }

    // 读取非线性优化器参数
    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    
    // 读取关键帧选择的最小视差阈值（像素），并转换为归一化平面上的弧度
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;  // 除以焦距，得到归一化坐标

    // 读取输出路径相关参数
    fsSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";  // 构建轨迹结果文件路径
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    
    // 创建输出文件（如果已存在则清空）
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    // 读取外参估计模式
    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    
    // 根据外参估计模式进行相应处理
    if (ESTIMATE_EXTRINSIC == 2)
    {
        // 模式2：无先验外参，需要完全标定
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());  // 初始化为单位矩阵
        TIC.push_back(Eigen::Vector3d::Zero());      // 初始化为零向量
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        // 模式0或1：有先验外参
        if (ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        // 从配置文件读取初始外参估计
        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;  // 读取相机到IMU的变换矩阵
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);  // 将OpenCV矩阵转换为Eigen矩阵
        
        // 提取旋转和平移部分
        RIC.push_back(T.block<3, 3>(0, 0));  // 取左上角3x3旋转矩阵
        TIC.push_back(T.block<3, 1>(0, 3));  // 取右上角3x1平移向量
    } 
    
    // 读取相机数量
    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    // 验证相机数量是否合法（目前只支持1或2个相机）
    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);  // 如果非法，终止程序
    }

    // 构建相机配置文件路径
    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);  // 提取配置文件所在目录
    
    // 读取第一个相机的标定文件路径
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;  // 拼接完整路径
    CAM_NAMES.push_back(cam0Path);

    // 如果是双目系统，处理第二个相机
    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;  // 设置为双目模式
        
        // 读取第二个相机的标定文件路径
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        CAM_NAMES.push_back(cam1Path);
        
        // 读取第二个相机的外参
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    // 设置默认参数值
    INIT_DEPTH = 5.0;                  // 默认初始深度
    BIAS_ACC_THRESHOLD = 0.1;          // 默认加速度计偏置阈值
    BIAS_GYR_THRESHOLD = 0.1;          // 默认陀螺仪偏置阈值

    // 读取时间偏移相关参数
    TD = fsSettings["td"];             // 初始时间偏移估计
    ESTIMATE_TD = fsSettings["estimate_td"];  // 是否估计时间偏移
    
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    // 读取图像尺寸
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    // 如果不使用IMU，则固定外参和时间偏移
    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;  // 固定外参
        ESTIMATE_TD = 0;         // 不估计时间偏移
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }

    // 释放配置文件资源
    fsSettings.release();
}