/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "../utility/visualization.h"

// Estimator类的构造函数，初始化VINS-Fusion的状态估计器
Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");  // 输出ROS日志信息，表示初始化开始
    initThreadFlag = false;   // 初始化线程标志为false（尚未启动处理线程）
    clearState();             // 调用clearState函数，清空所有状态变量和缓冲区
}

// Estimator类的析构函数
Estimator::~Estimator()
{
    // 如果启用了多线程模式
    if (MULTIPLE_THREAD)
    {
        processThread.join();  // 等待处理线程结束（线程同步）
        printf("join thread \n");  // 打印线程加入信息
    }
}

// 清空所有状态变量和缓冲区，重置估计器到初始状态
void Estimator::clearState()
{
    mProcess.lock();  // 获取互斥锁，确保线程安全
    
    // 清空IMU数据缓冲区（加速度计）
    while(!accBuf.empty())
        accBuf.pop();
    
    // 清空IMU数据缓冲区（陀螺仪）
    while(!gyrBuf.empty())
        gyrBuf.pop();
    
    // 清空特征数据缓冲区
    while(!featureBuf.empty())
        featureBuf.pop();

    // 重置时间相关变量
    prevTime = -1;   // 上一帧时间戳设为-1（无效值）
    curTime = 0;     // 当前时间戳设为0
    openExEstimation = 0;  // 外部参数估计标志设为0（不估计）
    
    // 初始化位置和旋转
    initP = Eigen::Vector3d(0, 0, 0);  // 初始位置设为原点
    initR = Eigen::Matrix3d::Identity();  // 初始旋转设为单位矩阵（无旋转）
    
    inputImageCnt = 0;        // 输入图像计数重置为0
    initFirstPoseFlag = false;  // 初始化第一帧位姿标志设为false

    // 清空滑动窗口内的所有状态变量
    // WINDOW_SIZE是滑动窗口的大小（通常是10）
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();    // 旋转矩阵设为单位矩阵
        Ps[i].setZero();        // 位置向量设为零向量
        Vs[i].setZero();        // 速度向量设为零向量
        Bas[i].setZero();       // 加速度计偏置设为零向量
        Bgs[i].setZero();       // 陀螺仪偏置设为零向量
        
        // 清空每个帧对应的IMU数据缓冲区
        dt_buf[i].clear();                   // 时间间隔缓冲区
        linear_acceleration_buf[i].clear();  // 线性加速度缓冲区
        angular_velocity_buf[i].clear();     // 角速度缓冲区

        // 清理预积分器指针
        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];  // 释放预积分器内存
        }
        pre_integrations[i] = nullptr;  // 预积分器指针设为空
    }

    // 初始化相机外参（IMU到相机的变换）
    // NUM_OF_CAM是相机数量（单目为1，双目为2）
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();      // 平移向量设为零
        ric[i] = Matrix3d::Identity();  // 旋转矩阵设为单位矩阵
    }

    // 重置其他状态标志和变量
    first_imu = false;  // 第一个IMU数据标志设为false
    sum_of_back = 0;    // 后端优化中边缘化的帧数
    sum_of_front = 0;   // 前端处理的帧数
    frame_count = 0;    // 当前帧计数
    solver_flag = INITIAL;  // 求解器状态设为初始化阶段
    
    initial_timestamp = 0;  // 初始时间戳设为0
    all_image_frame.clear();  // 清空所有图像帧的缓存

    // 清理临时预积分器和边缘化信息
    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;  // 删除临时预积分器
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;  // 删除上一次的边缘化信息

    tmp_pre_integration = nullptr;          // 临时预积分器指针置空
    last_marginalization_info = nullptr;    // 边缘化信息指针置空
    last_marginalization_parameter_blocks.clear();  // 清空边缘化参数块

    f_manager.clearState();  // 调用特征管理器的clearState函数

    failure_occur = 0;  // 失败计数器重置为0

    mProcess.unlock();  // 释放互斥锁
}

// 设置估计器的参数（相机内参、外参、IMU参数等）
void Estimator::setParameter()
{
    mProcess.lock();  // 获取互斥锁，确保线程安全
    
    // 设置相机外参（IMU到相机的变换）
    // TIC和RIC是外部定义的相机外参数组
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];  // 设置平移向量
        ric[i] = RIC[i];  // 设置旋转矩阵
        
        // 打印相机外参信息
        cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
    }
    
    f_manager.setRic(ric);  // 将相机外参设置到特征管理器

    // 设置投影因子的信息矩阵（用于加权残差）
    // FOCAL_LENGTH是焦距，1.5是经验值
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    
    td = TD;  // 设置IMU和相机的时间差（time offset）
    g = G;    // 设置重力加速度向量
    cout << "set g " << g.transpose() << endl;  // 打印重力向量

    // 读取相机内参（从配置文件）
    featureTracker.readIntrinsicParameter(CAM_NAMES);

    // 打印多线程配置
    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    
    // 如果启用了多线程且线程尚未初始化
    if (MULTIPLE_THREAD && !initThreadFlag)
    {
        initThreadFlag = true;  // 设置初始化标志为true
        
        // 创建处理线程，指向processMeasurements成员函数
        // this指针用于访问当前对象的成员函数
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    
    mProcess.unlock();  // 释放互斥锁
}

// 改变传感器配置（是否使用IMU和双目相机）
// 参数：
//   use_imu: 是否使用IMU（1使用，0不使用）
//   use_stereo: 是否使用双目（1使用，0不使用）
void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;  // 重启系统标志，如果传感器配置改变需要重启系统
    mProcess.lock();  // 获取互斥锁，保证线程安全
    
    // 检查传感器配置的有效性：至少使用两种传感器（视觉+IMU或双目）
    if(!use_imu && !use_stereo)
        printf("at least use two sensors! \n");  // 输出警告信息
    else
    {
        // 检查IMU配置是否发生变化
        if(USE_IMU != use_imu)
        {
            USE_IMU = use_imu;  // 更新IMU使用标志
            
            // 如果新配置要使用IMU（之前没有使用）
            if(USE_IMU)
            {
                // 重新使用IMU，需要重启系统
                restart = true;
            }
            else
            {
                // 如果不使用IMU了，清理与IMU相关的资源
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;  // 删除边缘化信息

                // 重置相关指针和容器
                tmp_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }
        
        // 更新双目配置
        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);  // 打印新的传感器配置
    }
    mProcess.unlock();  // 释放互斥锁
    
    // 如果需要重启系统
    if(restart)
    {
        clearState();   // 清空所有状态
        setParameter(); // 重新设置参数
    }
}

// 输入图像数据的主接口函数
// 参数：
//   t: 图像时间戳
//   _img: 左目图像
//   _img1: 右目图像（可选）
void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    inputImageCnt++;  // 图像计数器递增，用于统计处理了多少帧图像
    
    // 存储特征跟踪结果的容器
    // 类型：map<特征点ID, vector<pair<相机ID, 7维特征向量>>>
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    
    TicToc featureTrackerTime;  // 计时器，用于测量特征跟踪耗时

    // 调用特征跟踪器的trackImage函数
    // 根据是否有右目图像，调用不同版本的重载函数
    if(_img1.empty())
        featureFrame = featureTracker.trackImage(t, _img);      // 单目版本
    else
        featureFrame = featureTracker.trackImage(t, _img, _img1); // 双目版本
    
    // 输出特征跟踪耗时（被注释掉了）
    // printf("featureTracker time: %f\n", featureTrackerTime.toc());

    // 如果需要显示跟踪图像（调试模式）
    if (SHOW_TRACK)
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();  // 获取跟踪可视化图像
        pubTrackImage(imgTrack, t);  // 发布跟踪图像（通常是ROS话题）
    }
    
    // 根据多线程配置选择不同的处理方式
    if(MULTIPLE_THREAD)  // 多线程模式
    {     
        // 只在偶数帧时处理，跳帧处理以平衡计算负载
        if(inputImageCnt % 2 == 0)
        {
            mBuf.lock();  // 获取缓冲区锁
            // 将特征帧和时间戳推入缓冲区
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();  // 释放缓冲区锁
        }
    }
    else  // 单线程模式
    {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));  // 推入缓冲区
        mBuf.unlock();
        
        TicToc processTime;  // 计时器，测量处理耗时
        processMeasurements();  // 立即处理测量数据（同步模式）
        printf("process time: %f\n", processTime.toc());  // 输出处理耗时
    }
}

// 输入IMU数据的接口函数
// 参数：
//   t: IMU数据时间戳
//   linearAcceleration: 线性加速度（m/s^2）
//   angularVelocity: 角速度（rad/s）
void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();  // 获取缓冲区锁，保证线程安全
    
    // 将IMU数据推入缓冲区
    accBuf.push(make_pair(t, linearAcceleration));  // 加速度数据
    gyrBuf.push(make_pair(t, angularVelocity));     // 角速度数据
    
    //printf("input imu with time %f \n", t);  // 调试输出，被注释掉了
    mBuf.unlock();  // 释放缓冲区锁

    // 如果当前处于非线性优化状态（系统已经初始化完成）
    if (solver_flag == NON_LINEAR)
    {
        mPropagate.lock();  // 获取传播锁，防止状态传播过程中的数据竞争
        
        // 快速预测：使用最新的IMU数据快速更新当前状态，用于低延迟输出
        fastPredictIMU(t, linearAcceleration, angularVelocity);
        
        // 发布最新的里程计信息（低延迟，未经过优化但响应快）
        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
        
        mPropagate.unlock();  // 释放传播锁
    }
}

// 输入特征数据的接口函数（来自特征跟踪器）
// 参数：
//   t: 特征数据时间戳
//   featureFrame: 特征帧数据
void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame)
{
    mBuf.lock();  // 获取缓冲区锁
    featureBuf.push(make_pair(t, featureFrame));  // 将特征数据推入缓冲区
    mBuf.unlock();  // 释放缓冲区锁

    // 如果不是多线程模式，立即处理测量数据
    if(!MULTIPLE_THREAD)
        processMeasurements();
}

// 获取指定时间区间内的IMU数据
// 参数：
//   t0: 起始时间
//   t1: 结束时间
//   accVector: 输出参数，存储加速度数据
//   gyrVector: 输出参数，存储角速度数据
// 返回值：成功获取返回true，否则返回false
bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    // 检查加速度缓冲区是否为空
    if(accBuf.empty())
    {
        printf("not receive imu\n");  // 输出警告信息
        return false;  // 没有接收到IMU数据
    }
    
    //printf("get imu from %f %f\n", t0, t1);  // 调试输出
    //printf("imu front time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);  // 调试输出
    
    // 检查缓冲区中是否有足够的数据覆盖所需时间区间
    if(t1 <= accBuf.back().first)  // 结束时间小于等于缓冲区中最后一条数据的时间
    {
        // 移除缓冲区中时间早于t0的数据（过时数据）
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();  // 弹出加速度数据
            gyrBuf.pop();   // 弹出角速度数据
        }
        
        // 收集时间在[t0, t1)区间内的数据
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());  // 添加到加速度向量
            accBuf.pop();                         // 从缓冲区移除
            gyrVector.push_back(gyrBuf.front());  // 添加到角速度向量
            gyrBuf.pop();                         // 从缓冲区移除
        }
        
        // 添加时间恰好为t1的数据点（确保包含边界点）
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else  // 缓冲区数据不足以覆盖所需时间区间
    {
        printf("wait for imu\n");  // 输出等待信息
        return false;  // 获取失败
    }
    return true;  // 成功获取
}

// 检查在时间t是否有可用的IMU数据
// 参数：t - 要检查的时间
// 返回值：如果缓冲区不为空且t小于等于缓冲区最后一条数据的时间，返回true
bool Estimator::IMUAvailable(double t)
{
    // 检查缓冲区是否为空且时间是否在范围内
    if(!accBuf.empty() && t <= accBuf.back().first)
        return true;  // 有可用数据
    else
        return false;  // 无可用数据
}

// 处理测量数据的主循环（多线程模式下在独立线程中运行）
void Estimator::processMeasurements()
{
    while (1)  // 无限循环，持续处理数据
    {
        //printf("process measurments\n");  // 调试输出
        
        // 定义变量存储特征数据和IMU数据
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        
        // 如果特征缓冲区不为空
        if(!featureBuf.empty())
        {
            // 获取缓冲区中的第一个特征帧
            feature = featureBuf.front();
            curTime = feature.first + td;  // 当前时间 = 特征时间 + IMU-相机时间差
            
            // 等待IMU数据可用（如果使用IMU）
            while(1)
            {
                // 如果不使用IMU，或者IMU数据已经可用，则跳出循环
                if ((!USE_IMU  || IMUAvailable(feature.first + td)))
                    break;
                else  // IMU数据不可用，等待
                {
                    printf("wait for imu ... \n");  // 输出等待信息
                    
                    // 如果是单线程模式，直接返回（下次再处理）
                    if (! MULTIPLE_THREAD)
                        return;
                    
                    // 多线程模式下，休眠5毫秒后再次检查
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            
            mBuf.lock();  // 获取缓冲区锁
            
            // 如果使用IMU，获取从上一帧到当前帧之间的IMU数据
            if(USE_IMU)
                getIMUInterval(prevTime, curTime, accVector, gyrVector);
            
            // 从缓冲区中移除已处理的特征数据
            featureBuf.pop();
            mBuf.unlock();  // 释放缓冲区锁
            
            // 如果使用IMU，处理IMU数据
            if(USE_IMU)
            {
                // 如果尚未初始化第一帧位姿，使用IMU数据初始化
                if(!initFirstPoseFlag)
                    initFirstIMUPose(accVector);
                
                // 遍历所有IMU数据点
                for(size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;  // 时间间隔
                    
                    // 计算每个IMU数据点的时间间隔
                    if(i == 0)  // 第一个点：与上一帧特征时间的时间差
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)  // 最后一个点：与当前帧特征时间的时间差
                        dt = curTime - accVector[i - 1].first;
                    else  // 中间点：与前一个IMU点的时间差
                        dt = accVector[i].first - accVector[i - 1].first;
                    
                    // 处理单个IMU数据点
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }
            
            mProcess.lock();  // 获取处理锁
            
            // 处理图像特征数据（VINS核心算法）
            processImage(feature.second, feature.first);
            
            // 更新上一帧时间为当前时间，为下一帧处理做准备
            prevTime = curTime;
            
            // 打印统计信息
            printStatistics(*this, 0);
            
            // 创建ROS消息头
            std_msgs::Header header;
            header.frame_id = "world";  // 世界坐标系
            header.stamp = ros::Time(feature.first);  // 时间戳
            
            // 发布各种ROS话题
            pubOdometry(*this, header);     // 发布里程计
            pubKeyPoses(*this, header);     // 发布关键帧位姿
            pubCameraPose(*this, header);   // 发布相机位姿
            pubPointCloud(*this, header);   // 发布点云
            pubKeyframe(*this);             // 发布关键帧信息
            pubTF(*this, header);           // 发布TF变换
            
            mProcess.unlock();  // 释放处理锁
        }
        
        // 如果是单线程模式，处理完一帧后跳出循环
        if (! MULTIPLE_THREAD)
            break;
        
        // 多线程模式下，休眠2毫秒，避免CPU空转
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


// 使用IMU加速度数据初始化第一帧的位姿（主要是重力对齐）
// 参数：accVector - 包含时间戳和加速度向量的序列
void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");  // 输出初始化信息
    initFirstPoseFlag = true;         // 设置初始化标志为true，表示已初始化第一帧位姿
    //return;  // 如果取消注释，会直接返回，不执行后续初始化（用于调试）
    
    Eigen::Vector3d averAcc(0, 0, 0);  // 平均加速度向量，初始化为零
    int n = (int)accVector.size();     // 获取加速度向量的大小（IMU测量次数）
    
    // 计算所有IMU加速度测量值的平均值
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;  // 累加所有加速度向量
    }
    averAcc = averAcc / n;  // 计算平均加速度
    
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());  // 输出平均加速度
    
    // 将平均加速度向量（重力方向）转换为旋转矩阵
    // 这个函数假设初始时刻IMU的z轴与重力方向对齐
    Matrix3d R0 = Utility::g2R(averAcc);
    
    // 提取旋转矩阵的偏航角（yaw）
    double yaw = Utility::R2ypr(R0).x();
    
    // 去除偏航角的影响，只保留俯仰和滚转，使初始姿态的偏航角为0
    // 这是为了保证初始姿态的稳定性，避免初始时刻就有大的偏航角
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    
    // 将计算得到的旋转矩阵赋给滑动窗口的第一帧
    Rs[0] = R0;
    
    cout << "init R0 " << endl << Rs[0] << endl;  // 输出初始旋转矩阵
    
    // 可选的：初始化第一帧的速度（通常设为0，这里被注释掉了）
    //Vs[0] = Vector3d(5, 0, 0);
}

// 手动设置第一帧的位姿（位置和旋转）
// 参数：
//   p - 第一帧的位置
//   r - 第一帧的旋转矩阵
void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;  // 设置第一帧的位置
    Rs[0] = r;  //设置第一帧的旋转
    
    initP = p;  // 保存初始位置（可能用于其他用途）
    initR = r;  // 保存初始旋转（可能用于其他用途）
}

// 处理单次IMU测量数据
// 参数：
//   t - IMU测量时间戳
//   dt - 时间间隔（与上一次IMU测量的时间差）
//   linear_acceleration - 线性加速度测量值
//   angular_velocity - 角速度测量值
void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    // 如果是第一个IMU数据，记录加速度和角速度的初始值
    if (!first_imu)
    {
        first_imu = true;                    // 设置标志，表示已经接收到第一个IMU数据
        acc_0 = linear_acceleration;         // 记录初始加速度
        gyr_0 = angular_velocity;            // 记录初始角速度
    }

    // 如果当前帧的预积分器不存在，创建一个新的预积分器
    if (!pre_integrations[frame_count])
    {
        // 使用初始测量值、当前帧的偏置估计创建预积分器
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    
    // 如果不是第一帧（frame_count为0时是第一帧）
    if (frame_count != 0)
    {
        // 将当前IMU数据添加到当前帧的预积分器中
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        
        // 如果不是非线性优化状态，也将数据添加到临时预积分器（被注释掉了）
        //if(solver_flag != NON_LINEAR)
        //    tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        // 将IMU数据存入缓冲区（用于后续优化）
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;  // 当前帧的索引
        
        // 计算上一个IMU时刻的加速度（去除偏置和重力影响）
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        
        // 计算平均角速度（去除偏置影响）
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        
        // 更新旋转：使用中值积分法，将角速度转换为旋转矩阵增量
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        
        // 计算当前IMU时刻的加速度（去除偏置和重力影响）
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        
        // 使用两个时刻加速度的平均值
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        
        // 更新位置：使用中值积分法
        // 位置增量 = 速度*dt + 0.5*加速度*dt^2
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        
        // 更新速度：速度增量 = 加速度*dt
        Vs[j] += dt * un_acc;
    }
    
    // 更新上一个IMU测量值，用于下一次计算
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

// 处理图像特征的核心函数，VINS系统的核心
// 参数：
//   image: 特征帧数据，包含特征点ID、相机ID和7维特征向量
//   header: 图像时间戳
void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header)
{
    ROS_DEBUG("new image coming ------------------------------------------");  // 调试信息，新图像到达
    ROS_DEBUG("Adding feature points %lu", image.size());  // 调试信息，添加的特征点数量
    
    // 添加特征点到特征管理器，并检查视差决定是否为关键帧
    // addFeatureCheckParallax返回true表示有足够视差（关键帧），false表示没有足够视差（非关键帧）
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
    {
        marginalization_flag = MARGIN_OLD;  // 边缘化老的关键帧
        //printf("keyframe\n");  // 调试输出
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;  // 边缘化第二新的帧（如果视差不足）
        //printf("non-keyframe\n");  // 调试输出
    }

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");  // 输出关键帧/非关键帧信息
    ROS_DEBUG("Solving %d", frame_count);  // 输出当前帧计数
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());  // 输出特征点总数
    
    Headers[frame_count] = header;  // 存储当前帧的时间戳

    // 创建图像帧对象，包含特征数据和时间戳
    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;  // 设置预积分器（从上一帧到当前帧的IMU预积分）
    
    // 将图像帧插入到所有图像帧的map中，用于初始化
    all_image_frame.insert(make_pair(header, imageframe));
    
    // 创建新的临时预积分器，用于下一帧的IMU预积分
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    // 如果外参估计模式为2（需要在线标定）
    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");  // 输出标定信息
        
        // 如果不是第一帧
        if (frame_count != 0)
        {
            // 获取相邻两帧之间的特征点对应关系
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;  // 标定得到的IMU到相机的旋转
            
            // 调用外部旋转标定函数
            // 输入：特征点对应关系、预积分的旋转变化、输出：标定的旋转矩阵
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");  // 标定成功
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);  // 输出标定结果
                
                ric[0] = calib_ric;   // 更新IMU到相机的旋转矩阵
                RIC[0] = calib_ric;   // 更新全局的外参旋转矩阵
                ESTIMATE_EXTRINSIC = 1;  // 将外参估计模式改为1（已标定，后续只做微调）
            }
        }
    }

    // 如果系统还处于初始化阶段
    if (solver_flag == INITIAL)
    {
        // 单目+IMU的初始化
        if (!STEREO && USE_IMU)
        {
            // 当滑动窗口满了（达到WINDOW_SIZE）时开始初始化
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;  // 初始化结果标志
                
                // 如果不需要标定外参且时间间隔足够（大于0.1秒）
                if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    // 执行初始化结构（视觉惯性对齐）
                    result = initialStructure();
                    initial_timestamp = header;  // 更新初始时间戳
                }
                
                if(result)  // 如果初始化成功
                {
                    optimization();      // 执行优化
                    updateLatestStates(); // 更新最新状态
                    solver_flag = NON_LINEAR;  // 设置求解器状态为非线性优化
                    slideWindow();       // 滑动窗口
                    ROS_INFO("Initialization finish!");  // 输出初始化完成信息
                }
                else  // 初始化失败
                    slideWindow();  // 滑动窗口，移除最老的帧
            }
        }

        // 双目+IMU的初始化
        if(STEREO && USE_IMU)
        {
            // 使用PnP（透视n点）初始化当前帧的位姿
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            
            // 对特征点进行三角化，得到3D位置
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            
            // 当滑动窗口满了时开始初始化
            if (frame_count == WINDOW_SIZE)
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                
                // 更新所有图像帧的位姿
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];  // 旋转
                    frame_it->second.T = Ps[i];  // 平移
                    i++;
                }
                
                // 求解陀螺仪偏置
                solveGyroscopeBias(all_image_frame, Bgs);
                
                // 重新传播所有预积分器（使用新的偏置）
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                }
                
                optimization();       // 执行优化
                updateLatestStates();  // 更新最新状态
                solver_flag = NON_LINEAR;  // 设置求解器状态
                slideWindow();        // 滑动窗口
                ROS_INFO("Initialization finish!");  // 输出初始化完成信息
            }
        }

        // 仅双目的初始化（无IMU）
        if(STEREO && !USE_IMU)
        {
            // 使用PnP初始化位姿
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            
            // 三角化特征点
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            
            optimization();  // 执行优化

            // 当滑动窗口满了时
            if(frame_count == WINDOW_SIZE)
            {
                optimization();       // 再次优化
                updateLatestStates();  // 更新最新状态
                solver_flag = NON_LINEAR;  // 设置求解器状态
                slideWindow();        // 滑动窗口
                ROS_INFO("Initialization finish!");  // 输出初始化完成信息
            }
        }

        // 如果滑动窗口还没满，扩展窗口
        if(frame_count < WINDOW_SIZE)
        {
            frame_count++;  // 增加帧计数
            int prev_frame = frame_count - 1;  // 前一帧索引
            
            // 将前一帧的状态复制到当前帧（作为初始值）
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }

    }
    else  // 已经完成初始化，进入正常的非线性优化状态
    {
        TicToc t_solve;  // 计时器，测量求解时间
        
        // 如果不使用IMU，使用PnP初始化当前帧位姿
        if(!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        
        // 三角化新的特征点
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        
        optimization();  // 执行非线性优化
        
        // 剔除外点
        set<int> removeIndex;  // 需要移除的特征点ID集合
        outliersRejection(removeIndex);  // 外点剔除
        
        // 从特征管理器中移除外点
        f_manager.removeOutlier(removeIndex);
        
        // 如果不是多线程模式
        if (! MULTIPLE_THREAD)
        {
            // 从特征跟踪器中移除外点
            featureTracker.removeOutliers(removeIndex);
            
            // 预测特征点在下一帧的位置（用于特征跟踪）
            predictPtsInNextFrame();
        }
            
        ROS_DEBUG("solver costs: %fms", t_solve.toc());  // 输出求解时间

        // 失败检测
        if (failureDetection())
        {
            ROS_WARN("failure detection!");  // 输出失败警告
            failure_occur = 1;  // 设置失败标志
            
            // 清空状态并重新设置参数（重启系统）
            clearState();
            setParameter();
            
            ROS_WARN("system reboot!");  // 输出系统重启信息
            return;  // 返回，不继续处理
        }

        slideWindow();  // 滑动窗口
        
        f_manager.removeFailures();  // 移除失败的特征点
        
        // 准备VINS的输出数据
        key_poses.clear();  // 清空关键帧位姿
        
        // 收集滑动窗口中所有帧的位置
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        // 保存最新和最老的位姿（用于边缘化）
        last_R = Rs[WINDOW_SIZE];  // 最新帧的旋转
        last_P = Ps[WINDOW_SIZE];  // 最新帧的位置
        last_R0 = Rs[0];           // 最老帧的旋转
        last_P0 = Ps[0];           // 最老帧的位置
        
        updateLatestStates();  // 更新最新状态（用于低延迟输出）
    }  
}

// VINS单目+IMU初始化的核心函数
// 功能：通过视觉结构恢复（SFM）和视觉惯性对齐，初始化系统状态
// 返回值：true表示初始化成功，false表示失败
bool Estimator::initialStructure()
{
    TicToc t_sfm;  // 计时器，用于测量SFM过程的耗时
    
    // 检查IMU的可观测性（确保有足够的运动激励）
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;  // 累加所有帧的加速度变化
        
        // 遍历所有图像帧（从第二帧开始）
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;  // 当前帧预积分的时间总和
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;  // 平均加速度变化
            sum_g += tmp_g;  // 累加
        }
        
        Vector3d aver_g;  // 平均加速度变化
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);  // 计算平均值
        
        double var = 0;  // 方差，用于衡量加速度变化的离散程度
        
        // 计算方差
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);  // 计算平方差
            //cout << "frame g " << tmp_g.transpose() << endl;  // 调试输出
        }
        
        var = sqrt(var / ((int)all_image_frame.size() - 1));  // 计算标准差
        //ROS_WARN("IMU variation %f!", var);  // 输出IMU变化量
        
        // 如果标准差小于0.25，说明IMU运动激励不足（运动太小）
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");  // 输出警告信息
            //return false;  // 可以选择返回失败，但这里被注释掉了
        }
    }
    
    // 全局视觉结构恢复（Global SFM）
    Quaterniond Q[frame_count + 1];  // 存储每帧的旋转（四元数）
    Vector3d T[frame_count + 1];     // 存储每帧的位置
    map<int, Vector3d> sfm_tracked_points;  // SFM恢复的3D点（ID->3D坐标）
    vector<SFMFeature> sfm_f;  // 用于SFM的特征点列表
    
    // 将特征管理器中的特征点转换为SFM需要的格式
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;  // 特征点起始帧的前一帧索引
        SFMFeature tmp_feature;  // 创建SFM特征点对象
        tmp_feature.state = false;  // 初始状态设为false（未三角化）
        tmp_feature.id = it_per_id.feature_id;  // 特征点ID
        
        // 遍历特征点在各个帧中的观测
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;  // 帧索引递增
            Vector3d pts_j = it_per_frame.point;  // 特征点归一化平面坐标
            // 添加观测：(帧索引, 归一化平面坐标)
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);  // 添加到SFM特征点列表
    } 
    
    // 寻找两帧之间具有足够视差的帧对，并计算相对位姿
    Matrix3d relative_R;  // 相对旋转
    Vector3d relative_T;  // 相对平移
    int l;  // 选中的参考帧索引
    
    // 调用relativePose函数寻找合适的帧对
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");  // 提示运动不足
        return false;  // 初始化失败
    }
    
    // 执行全局SFM（Structure from Motion）
    GlobalSFM sfm;  // 创建全局SFM对象
    // 调用SFM的construct函数，恢复所有帧的位姿和3D点
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");  // SFM失败
        marginalization_flag = MARGIN_OLD;  // 设置边缘化标志为边缘化最老帧
        return false;  // 初始化失败
    }

    // 使用PnP为所有帧求解位姿（对于非关键帧）
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin();  // 从第一帧开始
    
    // 遍历所有图像帧
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
    {
        // 提供初始猜测
        cv::Mat r, rvec, t, D, tmp_r;
        
        // 如果是关键帧（已经在SFM中计算了位姿）
        if((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;  // 标记为关键帧
            
            // 将SFM得到的位姿从相机坐标系转换到IMU坐标系
            // Q[i]是相机坐标系到世界坐标系的旋转，RIC[0]是IMU到相机的旋转
            // 所以：R_imu_to_world = R_camera_to_world * R_imu_to_camera^T
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];  // 位置（相机坐标系下的）
            
            i++;  // 关键帧索引递增
            continue;  // 继续下一帧
        }
        
        // 如果当前帧的时间戳大于关键帧时间戳（说明跳过了某些关键帧）
        if((frame_it->first) > Headers[i])
        {
            i++;  // 跳过这个关键帧
        }
        
        // 对于非关键帧，使用PnP求解位姿
        
        // 计算初始猜测：使用最近关键帧的位姿
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        
        // 将Eigen矩阵转换为OpenCV矩阵
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);  // 旋转矩阵转换为旋转向量
        cv::eigen2cv(P_inital, t);   // 平移向量

        frame_it->second.is_key_frame = false;  // 标记为非关键帧
        
        // 准备PnP需要的3D点和2D点
        vector<cv::Point3f> pts_3_vector;  // 3D点（世界坐标系）
        vector<cv::Point2f> pts_2_vector;  // 2D点（归一化平面坐标）
        
        // 遍历当前帧的所有特征点
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;  // 特征点ID
            
            for (auto &i_p : id_pts.second)
            {
                // 在SFM恢复的3D点中查找当前特征点
                it = sfm_tracked_points.find(feature_id);
                
                // 如果找到了（该特征点已被三角化）
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;  // 世界坐标系下的3D点
                    
                    // 转换为OpenCV格式
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    
                    Vector2d img_pts = i_p.second.head<2>();  // 归一化平面坐标
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        
        // 创建相机内参矩阵（单位矩阵，因为使用归一化坐标）
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        
        // 检查是否有足够的点进行PnP求解（至少6个点）
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;  // 初始化失败
        }
        
        // 使用OpenCV的solvePnP函数求解位姿
        // 参数：3D点，2D点，相机内参，畸变系数，输出旋转向量，输出平移向量，使用迭代法
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");  // PnP求解失败
            return false;  // 初始化失败
        }
        
        // 将旋转向量转换为旋转矩阵
        cv::Rodrigues(rvec, r);
        
        MatrixXd R_pnp, tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);  // OpenCV矩阵转换为Eigen矩阵
        R_pnp = tmp_R_pnp.transpose();  // 转置（OpenCV和Eigen的坐标系差异）
        
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);  // 平移向量转换
        T_pnp = R_pnp * (-T_pnp);  // 调整平移向量
        
        // 将相机坐标系下的位姿转换到IMU坐标系
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    
    // 视觉惯性对齐：将视觉SFM结果与IMU预积分对齐
    if (visualInitialAlign())
        return true;  // 对齐成功，初始化完成
    else
    {
        ROS_INFO("misalign visual structure with IMU");  // 对齐失败
        return false;  // 初始化失败
    }
}

// 视觉惯性对齐函数：将视觉SFM结果与IMU预积分对齐，恢复尺度、重力方向、速度和偏置
// 返回值：true表示对齐成功，false表示失败
bool Estimator::visualInitialAlign()
{
    TicToc t_g;  // 计时器，用于测量对齐过程耗时
    VectorXd x;   // 优化变量向量，包含速度、重力向量、尺度等
    
    // 调用视觉IMU对齐函数，求解尺度、重力方向、速度和陀螺仪偏置
    // 参数：all_image_frame（所有图像帧），Bgs（陀螺仪偏置），g（重力向量），x（优化变量）
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    
    if(!result)  // 如果对齐失败
    {
        ROS_DEBUG("solve g failed!");  // 输出调试信息
        return false;  // 返回失败
    }

    // 更新滑动窗口中的状态（位置和旋转）
    for (int i = 0; i <= frame_count; i++)  // 遍历所有帧
    {
        // 从all_image_frame中获取第i帧的旋转和平移（视觉SFM的结果）
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        
        Ps[i] = Pi;  // 更新位置
        Rs[i] = Ri;  // 更新旋转
        
        all_image_frame[Headers[i]].is_key_frame = true;  // 标记为关键帧
    }

    double s = (x.tail<1>())(0);  // 提取尺度因子（x的最后一个元素）
    
    // 重新传播预积分器（使用新估计的偏置）
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        // repropagate函数用新的偏置重新计算预积分
        // 第一个参数是加速度计偏置（这里设为0，因为只重新计算陀螺仪偏置的影响）
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    
    // 应用尺度因子，并将坐标系转换到IMU坐标系（去除相机外参影响）
    // 公式：P_imu = s * P_camera - R_camera * T_IMU_to_camera
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);  // 同时将第一帧设为原点
    
    // 提取速度估计（x中包含每帧的速度）
    int kv = -1;  // 速度索引
    map<double, ImageFrame>::iterator frame_i;
    
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)  // 如果是关键帧
        {
            kv++;  // 索引递增
            // 速度：V = R_camera_to_world * v_camera
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    // 将世界坐标系与重力方向对齐
    Matrix3d R0 = Utility::g2R(g);  // 根据重力向量计算旋转矩阵，使z轴与重力方向对齐
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();  // 计算当前旋转的偏航角
    
    // 去除偏航角的影响，使初始帧的偏航角为0（只有俯仰和滚转）
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    
    g = R0 * g;  // 在新的坐标系下更新重力向量
    
    // 计算旋转差异，用于将整个系统旋转到新的世界坐标系
    //Matrix3d rot_diff = R0 * Rs[0].transpose();  // 另一种计算方式，被注释掉了
    Matrix3d rot_diff = R0;  // 直接使用R0作为旋转差异
    
    // 将所有状态旋转到新的世界坐标系
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];  // 旋转位置
        Rs[i] = rot_diff * Rs[i];  // 旋转旋转矩阵
        Vs[i] = rot_diff * Vs[i];  // 旋转速度
    }
    
    ROS_DEBUG_STREAM("g0     " << g.transpose());  // 输出重力向量
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());  // 输出第一帧的欧拉角

    // 清除特征点的深度信息（因为尺度改变了，需要重新三角化）
    f_manager.clearDepth();
    
    // 在新的尺度和坐标系下重新三角化特征点
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    return true;  // 对齐成功
}

// 寻找具有足够视差的参考帧对，用于SFM初始化
// 参数：
//   relative_R: 输出参数，相对旋转矩阵
//   relative_T: 输出参数，相对平移向量
//   l: 输出参数，选中的参考帧索引
// 返回值：true表示找到合适的帧对，false表示没找到
bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // 寻找与最新帧有足够对应点且视差足够大的前一帧
    for (int i = 0; i < WINDOW_SIZE; i++)  // 遍历滑动窗口中的所有帧
    {
        vector<pair<Vector3d, Vector3d>> corres;  // 存储对应点对
        
        // 获取第i帧和第WINDOW_SIZE帧（最新帧）之间的特征点对应关系
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        
        // 如果对应点数量足够（大于20）
        if (corres.size() > 20)
        {
            double sum_parallax = 0;  // 总视差
            double average_parallax;   // 平均视差
            
            // 计算所有对应点的平均视差
            for (int j = 0; j < int(corres.size()); j++)
            {
                // 提取两个点在归一化平面的坐标
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                
                double parallax = (pts_0 - pts_1).norm();  // 计算视差（两点之间的距离）
                sum_parallax = sum_parallax + parallax;    // 累加视差
            }
            
            average_parallax = 1.0 * sum_parallax / int(corres.size());  // 计算平均视差
            
            // 检查视差是否足够（460是焦距的近似值，将归一化坐标的视差转换为像素视差）
            // 条件：像素视差>30，并且能成功求解相对位姿
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;  // 记录选中的帧索引
                
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", 
                         average_parallax * 460, l);  // 输出调试信息
                
                return true;  // 找到合适的帧对
            }
        }
    }
    return false;  // 没找到合适的帧对
}

// 将状态变量从Eigen向量形式转换为double数组，以便传入Ceres进行优化
void Estimator::vector2double()
{
    // 处理滑动窗口中每一帧的位姿（位置和旋转）
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        // 位置（3维）
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        
        // 旋转（四元数，4维）
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        // 如果使用IMU，还需要处理速度和偏置
        if(USE_IMU)
        {
            // 速度（3维）
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            // 加速度计偏置（3维）
            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            // 陀螺仪偏置（3维）
            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    // 处理相机外参（IMU到相机的变换）
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        // 平移（3维）
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        
        // 旋转（四元数，4维）
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    // 处理特征点的逆深度
    VectorXd dep = f_manager.getDepthVector();  // 获取所有特征点的逆深度向量
    
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);  // 每个特征点只有一个参数：逆深度

    // 处理时间偏移（IMU和相机之间的时间差）
    para_Td[0][0] = td;
}

// 将Ceres优化后的double数组参数转换回Eigen向量形式的状态变量
void Estimator::double2vector()
{
    // 保存第一帧的原始旋转（欧拉角）和位置，用于后续的坐标系对齐
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);  // 第一帧的旋转（欧拉角：yaw, pitch, roll）
    Vector3d origin_P0 = Ps[0];                  // 第一帧的位置

    // 如果之前发生了失败，则使用上一次成功的位姿作为原点
    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);  // 上一次成功的第0帧旋转
        origin_P0 = last_P0;                  // 上一次成功的第0帧位置
        failure_occur = 0;                    // 重置失败标志
    }

    // 如果使用IMU，需要处理坐标系对齐问题（因为优化过程中固定了第一帧，但可能存在漂移）
    if(USE_IMU)
    {
        // 从优化变量中提取第一帧的旋转（欧拉角）
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                          para_Pose[0][3],
                                                          para_Pose[0][4],
                                                          para_Pose[0][5]).toRotationMatrix());
        // 计算原始第一帧与优化后第一帧的偏航角差值
        double y_diff = origin_R0.x() - origin_R00.x();
        // TODO: 可能需要更完善的处理
        // 构建一个绕z轴旋转y_diff角度的旋转矩阵
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        
        // 处理万向锁奇异点：当俯仰角接近±90度时，欧拉角表示不唯一
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");  // 输出奇异点警告
            // 使用旋转矩阵直接计算旋转差异，避免欧拉角奇异问题
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5]).toRotationMatrix().transpose();
        }

        // 遍历滑动窗口中的所有帧
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            // 更新旋转：将优化后的旋转应用旋转差异，使其与原始坐标系对齐
            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            // 更新位置：将优化后的位置应用旋转差异，并加上原始第一帧的位置
            // 注意：优化时固定了第一帧的位置为0，所以这里需要减去para_Pose[0]再加上origin_P0
            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;

            // 更新速度：应用旋转差异
            Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                        para_SpeedBias[i][1],
                                        para_SpeedBias[i][2]);

            // 更新加速度计偏置
            Bas[i] = Vector3d(para_SpeedBias[i][3],
                              para_SpeedBias[i][4],
                              para_SpeedBias[i][5]);

            // 更新陀螺仪偏置
            Bgs[i] = Vector3d(para_SpeedBias[i][6],
                              para_SpeedBias[i][7],
                              para_SpeedBias[i][8]);
        }
    }
    else  // 如果不使用IMU（纯视觉）
    {
        // 纯视觉情况下，直接使用优化后的位姿
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    // 如果使用IMU，更新相机外参
    if(USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5]).normalized().toRotationMatrix();
        }
    }

    // 更新特征点的逆深度
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];  // 将优化后的逆深度赋值
    f_manager.setDepth(dep);  // 将新的逆深度设置回特征管理器

    // 如果使用IMU，更新时间偏移
    if(USE_IMU)
        td = para_Td[0][0];
}

// 失败检测：检查系统是否出现异常状态
bool Estimator::failureDetection()
{
    // 第一行直接返回false，意味着失败检测被禁用（可能是调试模式）
    return false;
    
    // 如果跟踪的特征点数量太少
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;  // 可以返回true，但被注释掉了
    }
    
    // 如果加速度计偏置估计过大（超过2.5）
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;  // 返回true表示检测到失败
    }
    
    // 如果陀螺仪偏置估计过大（超过1.0）
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;  // 返回true表示检测到失败
    }
    
    /*
    // 如果相机外参的平移估计过大（被注释掉了）
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    
    // 检查最新帧的位置变化是否过大
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)  // 位置变化超过5米
    {
        //ROS_INFO(" big translation");
        //return true;  // 被注释掉了
    }
    
    // 检查最新帧的垂直方向变化是否过大
    if (abs(tmp_P.z() - last_P.z()) > 1)  // 垂直变化超过1米
    {
        //ROS_INFO(" big z translation");
        //return true;  // 被注释掉了
    }
    
    // 检查最新帧的旋转变化是否过大
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;  // 计算旋转差异
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;  // 计算角度变化（度）
    if (delta_angle > 50)  // 角度变化超过50度
    {
        ROS_INFO(" big delta_angle ");
        //return true;  // 被注释掉了
    }
    
    return false;  // 没有检测到失败
}

// VINS-Fusion系统的核心优化函数，使用Ceres进行非线性优化
void Estimator::optimization()
{
    TicToc t_whole, t_prepare;  // 计时器：整个优化过程和准备过程
    vector2double();  // 将状态变量从Eigen向量转换为Ceres需要的double数组

    // 创建Ceres优化问题
    ceres::Problem problem;
    ceres::LossFunction *loss_function;  // 损失函数指针
    //loss_function = NULL;  // 不使用损失函数（被注释掉了）
    loss_function = new ceres::HuberLoss(1.0);  // 使用Huber损失函数，参数为1.0
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);  // 另一种选择：Cauchy损失函数
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);  // 同上

    // 添加滑动窗口中所有帧的位姿参数块
    for (int i = 0; i < frame_count + 1; i++)
    {
        // 创建位姿局部参数化（处理四元数的更新）
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);  // 添加位姿参数块（7维）
        if(USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);  // 如果使用IMU，添加速度偏置参数块（9维）
    }
    
    // 如果不使用IMU，固定第一帧的位姿（消除尺度模糊性）
    if(!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    // 添加相机外参参数块
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);  // 添加相机外参参数块
        
        // 判断是否需要估计相机外参
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            //ROS_INFO("estimate extinsic param");
            openExEstimation = 1;  // 开启外参估计
        }
        else
        {
            //ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);  // 固定外参
        }
    }
    
    // 添加时间偏移参数块
    problem.AddParameterBlock(para_Td[0], 1);  // 添加时间偏移参数块（1维）
    
    // 判断是否需要估计时间偏移
    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);  // 固定时间偏移

    // 添加上一次边缘化的先验因子（如果存在）
    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // 创建边缘化因子
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        // 添加上一次边缘化的残差块
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    
    // 如果使用IMU，添加IMU因子
    if(USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;  // 下一帧索引
            if (pre_integrations[j]->sum_dt > 10.0)  // 如果预积分时间太长（超过10秒），跳过
                continue;
            // 创建IMU因子
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
            // 添加IMU残差块，连接两帧的位姿和速度偏置
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }

    // 添加视觉因子
    int f_m_cnt = 0;  // 视觉测量计数
    int feature_index = -1;  // 特征点索引
    
    // 遍历所有特征点
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();  // 记录特征点被观测的次数
        if (it_per_id.used_num < 4)  // 如果观测次数小于4，跳过（为了稳定性）
            continue;
 
        ++feature_index;  // 特征点索引递增

        int imu_i = it_per_id.start_frame;  // 特征点首次出现的帧
        int imu_j = imu_i - 1;  // 起始帧的前一帧
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;  // 首次观测的归一化坐标

        // 遍历特征点的所有观测
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;  // 当前帧索引
            if (imu_i != imu_j)  // 如果不是同一帧（帧间观测）
            {
                Vector3d pts_j = it_per_frame.point;  // 当前观测的归一化坐标
                // 创建双目（或单目）相机两帧之间的投影因子
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                // 添加残差块：连接两帧位姿、相机外参、特征点逆深度、时间偏移
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            // 如果是双目相机且当前观测是双目观测
            if(STEREO && it_per_frame.is_stereo)
            {                
                Vector3d pts_j_right = it_per_frame.pointRight;  // 右目观测的归一化坐标
                if(imu_i != imu_j)  // 帧间双目观测
                {
                    // 创建双目相机两帧之间的投影因子（左右目）
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    // 添加残差块：连接两帧位姿、两个相机外参、特征点逆深度、时间偏移
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else  // 帧内双目观测（同一时刻左右目）
                {
                    // 创建单帧双目投影因子
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    // 添加残差块：连接两个相机外参、特征点逆深度、时间偏移
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
            }
            f_m_cnt++;  // 视觉测量计数递增
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);  // 输出视觉测量数量
    //printf("prepare for ceres: %f \n", t_prepare.toc());  // 输出准备时间

    // 配置Ceres求解器选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;  // 使用DENSE_SCHUR线性求解器
    //options.num_threads = 2;  // 设置线程数（被注释掉了）
    options.trust_region_strategy_type = ceres::DOGLEG;  // 使用DOGLEG信赖域策略
    options.max_num_iterations = NUM_ITERATIONS;  // 最大迭代次数
    //options.use_explicit_schur_complement = true;  // 使用显式舒尔补（被注释掉了）
    //options.minimizer_progress_to_stdout = true;  // 输出优化进度（被注释掉了）
    //options.use_nonmonotonic_steps = true;  // 使用非单调步骤（被注释掉了）
    
    // 根据边缘化标志设置不同的求解时间限制
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;  // 边缘化旧帧时给更多时间
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;  // 边缘化新帧时正常时间
    
    TicToc t_solver;  // 求解计时器
    ceres::Solver::Summary summary;  // 求解器摘要
    ceres::Solve(options, &problem, &summary);  // 执行优化
    //cout << summary.BriefReport() << endl;  // 输出优化报告（被注释掉了）
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));  // 输出迭代次数
    //printf("solver costs: %f \n", t_solver.toc());  // 输出求解时间

    double2vector();  // 将优化结果转换回Eigen向量
    //printf("frame_count: %d \n", frame_count);  // 输出帧计数

    // 如果滑动窗口还没满，直接返回
    if(frame_count < WINDOW_SIZE)
        return;
    
    // 开始边缘化处理
    TicToc t_whole_marginalization;
    
    // 如果需要边缘化旧的关键帧（第0帧）
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();  // 创建边缘化信息对象
        vector2double();  // 再次转换，确保使用最新的状态

        // 添加上一次边缘化的先验因子（如果存在）
        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;  // 需要丢弃的参数块索引
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                // 如果参数块是第0帧的位姿或速度偏置，需要丢弃（被边缘化）
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // 创建边缘化因子
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            // 创建残差块信息
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);  // 添加到边缘化信息
        }

        // 添加第0帧和第1帧之间的IMU因子（如果要边缘化第0帧）
        if(USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)  // 确保预积分时间合理
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                // 创建IMU残差块信息，需要丢弃第0帧的位姿和速度偏置
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);  // 添加到边缘化信息
            }
        }

        // 添加与第0帧相关的视觉因子
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)  // 跳过观测太少的特征点
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)  // 只处理起始帧为第0帧的特征点
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                // 遍历特征点的所有观测
                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)  // 帧间观测
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        // 创建视觉残差块信息，需要丢弃第0帧位姿和特征点逆深度
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    // 双目观测处理
                    if(STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)  // 帧间双目
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else  // 帧内双目
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        // 执行预边缘化（计算残差和雅可比矩阵）
        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        // 执行边缘化（舒尔补操作）
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        // 创建地址偏移映射，用于更新参数块指针（因为滑动窗口移动了）
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];  // 位姿向前移动
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];  // 速度偏置向前移动
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];  // 相机外参保持不变

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];  // 时间偏移保持不变

        // 获取边缘化后的参数块
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        // 清理旧边缘化信息，更新为新的
        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else  // 如果需要边缘化第二新的帧（第WINDOW_SIZE-1帧）
    {
        // 只有当上一次边缘化的参数块中包含第WINDOW_SIZE-1帧的位姿时才执行边缘化
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {
            MarginalizationInfo *marginalization_info = new MarginalizationInfo();  // 创建边缘化信息对象
            vector2double();  // 转换状态变量

            // 添加上一次边缘化的先验因子
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;  // 需要丢弃的参数块索引
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);  // 确保不是速度偏置
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])  // 如果是第WINDOW_SIZE-1帧的位姿
                        drop_set.push_back(i);  // 需要丢弃
                }
                // 创建边缘化因子
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                // 创建残差块信息
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);
                marginalization_info->addResidualBlockInfo(residual_block_info);  // 添加到边缘化信息
            }

            // 执行预边缘化和边缘化
            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            // 创建地址偏移映射（比边缘化旧帧的情况更复杂）
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)  // 第WINDOW_SIZE-1帧被边缘化，跳过
                    continue;
                else if (i == WINDOW_SIZE)  // 第WINDOW_SIZE帧向前移动
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else  // 其他帧保持不变
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];  // 相机外参保持不变

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];  // 时间偏移保持不变

            // 获取边缘化后的参数块
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            
            // 清理旧边缘化信息，更新为新的
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());  // 输出边缘化总时间
    //printf("whole time for ceres: %f \n", t_whole.toc());  // 输出整个优化时间
}

// 滑动窗口管理函数：根据边缘化标志移动滑动窗口中的状态数据
void Estimator::slideWindow()
{
    TicToc t_margin;  // 计时器，用于测量滑动窗口操作的耗时
    
    // 如果是边缘化旧的关键帧（第0帧）
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];  // 保存第0帧的时间戳，用于从all_image_frame中删除
        back_R0 = Rs[0];          // 备份第0帧的旋转，可能用于失败恢复
        back_P0 = Ps[0];          // 备份第0帧的位置，可能用于失败恢复
        
        // 只有当滑动窗口满了（frame_count == WINDOW_SIZE）时才需要滑动
        if (frame_count == WINDOW_SIZE)
        {
            // 将第1到第WINDOW_SIZE帧向前移动一位（覆盖第0帧）
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];  // 时间戳前移
                Rs[i].swap(Rs[i + 1]);        // 旋转矩阵交换（前移）
                Ps[i].swap(Ps[i + 1]);        // 位置向量交换（前移）
                
                // 如果使用IMU，还需要交换IMU相关数据
                if(USE_IMU)
                {
                    // 交换预积分器指针
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);
                    
                    // 交换IMU数据缓冲区
                    dt_buf[i].swap(dt_buf[i + 1]);                     // 时间间隔缓冲区
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);  // 加速度缓冲区
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);        // 角速度缓冲区
                    
                    // 交换状态变量
                    Vs[i].swap(Vs[i + 1]);    // 速度
                    Bas[i].swap(Bas[i + 1]);  // 加速度计偏置
                    Bgs[i].swap(Bgs[i + 1]);  // 陀螺仪偏置
                }
            }
            
            // 处理新加入的帧（现在是第WINDOW_SIZE帧）
            // 用第WINDOW_SIZE-1帧的值初始化新帧（只是一个占位符，会被后续更新）
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            
            // 如果使用IMU，处理IMU相关数据
            if(USE_IMU)
            {
                // 用前一帧的值初始化新帧的状态
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];
                
                // 删除旧的预积分器（第WINDOW_SIZE帧的），创建新的
                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};
                
                // 清空新帧的IMU数据缓冲区
                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            
            // 从all_image_frame中删除被边缘化的帧及其之前的所有帧
            // 条件：总是执行（true）或处于初始化阶段
            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);  // 找到第0帧在all_image_frame中的位置
                
                // 删除第0帧的预积分器
                delete it_0->second.pre_integration;
                
                // 删除从开始到第0帧（包括第0帧）的所有图像帧
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            
            // 调用特征管理器的滑动窗口函数，更新特征点的起始帧索引
            slideWindowOld();
        }
    }
    else  // 如果是边缘化第二新的帧（第WINDOW_SIZE-1帧，即倒数第二帧）
    {
        // 只有当滑动窗口满了时才需要处理
        if (frame_count == WINDOW_SIZE)
        {
            // 将第WINDOW_SIZE帧（最新帧）的数据复制到第WINDOW_SIZE-1帧
            Headers[frame_count - 1] = Headers[frame_count];  // 时间戳
            Ps[frame_count - 1] = Ps[frame_count];           // 位置
            Rs[frame_count - 1] = Rs[frame_count];           // 旋转
            
            // 如果使用IMU，需要处理IMU数据
            if(USE_IMU)
            {
                // 将第WINDOW_SIZE帧的IMU数据转移到第WINDOW_SIZE-1帧的预积分器中
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];                            // 时间间隔
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];  // 加速度
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];        // 角速度
                    
                    // 将IMU数据添加到第WINDOW_SIZE-1帧的预积分器中
                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);
                    
                    // 将IMU数据添加到第WINDOW_SIZE-1帧的缓冲区中
                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }
                
                // 复制状态变量
                Vs[frame_count - 1] = Vs[frame_count];     // 速度
                Bas[frame_count - 1] = Bas[frame_count];   // 加速度计偏置
                Bgs[frame_count - 1] = Bgs[frame_count];   // 陀螺仪偏置
                
                // 删除第WINDOW_SIZE帧的预积分器，创建新的
                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};
                
                // 清空第WINDOW_SIZE帧的IMU缓冲区
                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            
            // 调用特征管理器的滑动窗口函数，更新特征点的起始帧索引
            slideWindowNew();
        }
    }
}

// 边缘化第二新的帧（倒数第二帧）时的滑动窗口操作
// 主要用于非关键帧情况，只更新前端统计和特征管理器
void Estimator::slideWindowNew()
{
    sum_of_front++;  // 前端统计计数器递增（统计前端处理次数）
    f_manager.removeFront(frame_count);  // 调用特征管理器的removeFront函数
                                         // 移除起始帧等于当前frame_count的特征点
                                         // 因为这些特征点是从被边缘化的帧开始跟踪的
}

// 边缘化最旧的帧（第0帧）时的滑动窗口操作
// 主要用于关键帧情况，需要更复杂的处理
void Estimator::slideWindowOld()
{
    sum_of_back++;  // 后端统计计数器递增（统计后端边缘化次数）

    // 判断是否需要调整特征点深度
    // 只有在非线性优化状态（已完成初始化）才需要调整深度
    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    
    if (shift_depth)  // 需要调整深度
    {
        Matrix3d R0, R1;  // 两个时刻的相机旋转矩阵
        Vector3d P0, P1;  // 两个时刻的相机位置向量
        
        // 计算被边缘化帧（旧的第0帧）的相机位姿
        // R0 = back_R0 * ric[0]: IMU旋转 × IMU到相机旋转 = 相机旋转
        R0 = back_R0 * ric[0];
        // P0 = back_P0 + back_R0 * tic[0]: IMU位置 + IMU旋转 × IMU到相机平移 = 相机位置
        P0 = back_P0 + back_R0 * tic[0];
        
        // 计算新第0帧（原来的第1帧）的相机位姿
        R1 = Rs[0] * ric[0];  // Rs[0]现在是原来的第1帧的IMU旋转
        P1 = Ps[0] + Rs[0] * tic[0];  // Ps[0]现在是原来的第1帧的IMU位置
        
        // 调用特征管理器的removeBackShiftDepth函数
        // 传入两帧的相机位姿，用于调整特征点的深度值
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else  // 不需要调整深度（初始化阶段）
        f_manager.removeBack();  // 简单移除与被边缘化帧相关的特征点
}

// 获取当前帧在世界坐标系下的位姿（4x4齐次变换矩阵）
// 参数：T - 输出参数，存储位姿矩阵
void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();  // 初始化为单位矩阵
    T.block<3, 3>(0, 0) = Rs[frame_count];  // 左上3x3旋转部分 = 当前帧旋转矩阵
    T.block<3, 1>(0, 3) = Ps[frame_count];  // 右上3x1平移部分 = 当前帧位置向量
}

// 获取指定帧在世界坐标系下的位姿（重载版本）
// 参数：
//   index - 帧索引
//   T - 输出参数，存储位姿矩阵
void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];  // 指定帧的旋转矩阵
    T.block<3, 1>(0, 3) = Ps[index];  // 指定帧的位置向量
}

// 预测特征点在下一帧的位置（用于辅助特征跟踪）
// 假设匀速运动模型，根据当前帧和前两帧的运动预测下一帧位姿
void Estimator::predictPtsInNextFrame()
{
    //printf("predict pts in next frame\n");  // 调试输出
    
    // 如果帧数小于2，没有足够的历史信息进行预测
    if(frame_count < 2)
        return;
    
    // 假设匀速运动模型来预测下一帧的位姿
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);  // 获取当前帧位姿
    getPoseInWorldFrame(frame_count - 1, prevT);  // 获取前一帧位姿
    
    // 预测下一帧位姿：假设当前帧到下一帧的变换 = 前一帧到当前帧的变换
    // 数学公式：nextT = curT * (prevT.inverse() * curT)
    // 等价于：从prevT到curT的变换为 ΔT = curT * prevT.inverse()
    // 则nextT = ΔT * curT = curT * prevT.inverse() * curT
    nextT = curT * (prevT.inverse() * curT);
    
    // 存储预测的特征点位置（特征点ID -> 在下一帧相机坐标系下的位置）
    map<int, Eigen::Vector3d> predictPts;

    // 遍历所有特征点
    for (auto &it_per_id : f_manager.feature)
    {
        // 只处理已经估计出深度的特征点
        if(it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;  // 特征点首次出现的帧索引
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;  // 最后出现的帧索引
            
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);  // 调试输出
            
            // 如果特征点被至少2帧观测到，并且最后出现在当前帧
            if((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;  // 特征点深度
                
                // 计算特征点在世界坐标系中的位置
                // 1. 在首次观测的相机坐标系中：depth * 归一化坐标
                // 2. 转换到IMU坐标系：ric[0] * (相机坐标) + tic[0]
                // 3. 转换到世界坐标系：Rs[firstIndex] * (IMU坐标) + Ps[firstIndex]
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                
                // 将世界坐标转换到预测的下一帧相机坐标系中
                // 1. 转换到下一帧IMU坐标系：R_next^T * (pts_w - t_next)
                // 2. 转换到下一帧相机坐标系：ric[0]^T * (IMU坐标 - tic[0])
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                
                int ptsIndex = it_per_id.feature_id;  // 特征点ID
                predictPts[ptsIndex] = pts_cam;  // 存储预测位置
            }
        }
    }
    
    // 将预测的位置设置给特征跟踪器，用于下一帧的特征跟踪
    featureTracker.setPrediction(predictPts);
    //printf("estimator output %d predict pts\n",(int)predictPts.size());  // 调试输出
}

// 计算重投影误差（用于外点剔除）
// 参数：
//   Ri, Pi: 第i帧的IMU旋转和位置
//   rici, tici: IMU到第i帧相机的旋转和平移
//   Rj, Pj: 第j帧的IMU旋转和位置
//   ricj, ticj: IMU到第j帧相机的旋转和平移
//   depth: 特征点在i帧中的深度
//   uvi: 特征点在i帧中的归一化坐标
//   uvj: 特征点在j帧中的归一化坐标（观测值）
// 返回值：重投影误差（像素距离）
double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj)
{
    // 将特征点从第i帧投影到世界坐标系
    // 1. i帧相机坐标：depth * uvi
    // 2. i帧IMU坐标：rici * (相机坐标) + tici
    // 3. 世界坐标：Ri * (IMU坐标) + Pi
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    
    // 将世界坐标投影到第j帧相机坐标系
    // 1. j帧IMU坐标：Rj^T * (pts_w - Pj)
    // 2. j帧相机坐标：ricj^T * (IMU坐标 - ticj)
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    
    // 计算重投影误差：投影坐标与观测坐标的差
    // pts_cj / pts_cj.z(): 归一化平面坐标
    // uvj.head<2>(): 观测的归一化坐标
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    
    double rx = residual.x();
    double ry = residual.y();
    
    // 返回误差的欧氏距离
    return sqrt(rx * rx + ry * ry);
}

// 外点剔除函数：基于重投影误差剔除异常的特征点
// 参数：removeIndex - 输出参数，存储需要移除的特征点ID集合
void Estimator::outliersRejection(set<int> &removeIndex)
{
    //return;  // 如果取消注释，会直接返回，不进行外点剔除（用于调试）
    
    int feature_index = -1;  // 特征点索引，用于遍历
    
    // 遍历特征管理器中的所有特征点
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;      // 累计误差
        int errCnt = 0;      // 误差计数（参与计算的观测次数）
        
        it_per_id.used_num = it_per_id.feature_per_frame.size();  // 记录特征点被观测的次数
        
        if (it_per_id.used_num < 4)  // 如果观测次数小于4，跳过（为了稳定性）
            continue;
        
        feature_index++;  // 特征点索引递增
        int imu_i = it_per_id.start_frame;  // 特征点首次出现的帧索引
        int imu_j = imu_i - 1;  // 用于遍历的帧索引（初始化为起始帧的前一帧）
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;  // 首次观测的归一化坐标
        double depth = it_per_id.estimated_depth;  // 特征点的估计深度
        
        // 遍历特征点的所有观测
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;  // 当前帧索引
            if (imu_i != imu_j)  // 如果不是同一帧（即帧间观测）
            {
                Vector3d pts_j = it_per_frame.point;  // 当前观测的归一化坐标
                
                // 计算重投影误差（左目到左目）
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                    Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;  // 累加误差
                errCnt++;          // 增加计数
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);  // 调试输出
            }
            
            // 如果是双目相机且当前观测是双目观测
            // 注意：需要重写投影因子（代码中的注释）
            if(STEREO && it_per_frame.is_stereo)
            {
                Vector3d pts_j_right = it_per_frame.pointRight;  // 右目观测的归一化坐标
                if(imu_i != imu_j)  // 帧间双目观测（左目到右目）
                {            
                    // 计算重投影误差（左目到右目）
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else  // 帧内双目观测（同一时刻左右目）
                {
                    // 计算重投影误差（左目到右目，同一时刻）
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }       
            }
        }
        
        // 计算平均重投影误差
        double ave_err = err / errCnt;
        
        // 如果平均误差大于阈值（3像素），将该特征点ID加入移除集合
        // 注意：误差是在归一化平面计算的，需要乘以焦距转换为像素误差
        if(ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);
    }
}

// 快速IMU预测函数：使用最新的IMU数据快速更新当前状态（用于低延迟输出）
// 参数：
//   t - IMU数据时间戳
//   linear_acceleration - 线性加速度测量值
//   angular_velocity - 角速度测量值
void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;  // 计算时间间隔
    latest_time = t;  // 更新时间戳
    
    // 中值积分法：计算上一个IMU时刻的加速度（去除偏置和重力影响）
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    
    // 计算平均角速度（去除偏置影响）
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    
    // 更新旋转：使用角速度积分
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    
    // 计算当前IMU时刻的加速度（去除偏置和重力影响）
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    
    // 使用两个时刻加速度的平均值
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    
    // 更新位置：使用中值积分法
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    
    // 更新速度
    latest_V = latest_V + dt * un_acc;
    
    // 保存当前IMU测量值，用于下一次预测
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

// 更新最新状态函数：将滑动窗口中的最新状态提取出来，并用未处理的IMU数据进行快速预测
// 用于提供低延迟的状态估计输出
void Estimator::updateLatestStates()
{
    mPropagate.lock();  // 获取传播锁，防止与快速IMU预测的数据竞争
    
    // 使用滑动窗口中最新帧的状态初始化快速预测的状态变量
    latest_time = Headers[frame_count] + td;  // 最新帧时间戳加上时间偏移
    latest_P = Ps[frame_count];    // 位置
    latest_Q = Rs[frame_count];    // 旋转（四元数形式，这里存储的是旋转矩阵，但fastPredictIMU需要四元数）
    latest_V = Vs[frame_count];    // 速度
    latest_Ba = Bas[frame_count];  // 加速度计偏置
    latest_Bg = Bgs[frame_count];  // 陀螺仪偏置
    latest_acc_0 = acc_0;          // 上一个IMU加速度测量值
    latest_gyr_0 = gyr_0;          // 上一个IMU角速度测量值
    
    // 获取缓冲区中未处理的IMU数据，用于快速预测到最新时刻
    mBuf.lock();  // 获取缓冲区锁
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;  // 复制加速度缓冲区
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;  // 复制角速度缓冲区
    mBuf.unlock();  // 释放缓冲区锁
    
    // 使用所有未处理的IMU数据进行快速预测
    while(!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;  // IMU数据时间戳
        Eigen::Vector3d acc = tmp_accBuf.front().second;  // 加速度
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;  // 角速度
        
        fastPredictIMU(t, acc, gyr);  // 快速IMU预测
        
        tmp_accBuf.pop();  // 弹出已处理的数据
        tmp_gyrBuf.pop();
    }
    
    mPropagate.unlock();  // 释放传播锁
}