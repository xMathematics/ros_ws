/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"
#include "../utility/tic_toc.h"

// 这是一个视觉SLAM/VIO中的特征点管理模块
// 主要包含三个类：FeaturePerFrame（单帧中的特征）、FeaturePerId（特征在多个帧中的跟踪）和FeatureManager（特征管理器）

// FeaturePerFrame类：表示单个特征在单个帧中的观测信息
class FeaturePerFrame
{
  public:
    // 构造函数：初始化特征在单个帧中的观测
    // 参数：
    //   _point: 7维向量，包含[三维点坐标x,y,z, 像素坐标u,v, 像素速度vx,vy]
    //   td: 时间延迟(time delay)或时间戳偏移
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        // 设置三维点坐标（在相机坐标系下）
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        
        // 设置像素坐标（归一化平面坐标或像素坐标）
        uv.x() = _point(3);
        uv.y() = _point(4);
        
        // 设置像素速度（光流或特征跟踪的速度）
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        
        // 存储时间延迟
        cur_td = td;
        
        // 标记为非双目观测（默认为单目）
        is_stereo = false;
    }
    
    // 设置右目观测（如果是双目相机）
    // 参数：
    //   _point: 7维向量，格式与构造函数相同，表示右目相机的观测
    void rightObservation(const Eigen::Matrix<double, 7, 1> &_point)
    {
        // 设置右目的三维点坐标
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        
        // 设置右目的像素坐标
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        
        // 设置右目的像素速度
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
        
        // 标记为双目观测
        is_stereo = true;
    }
    
    // 成员变量
    double cur_td;              // 时间延迟或时间戳偏移
    Vector3d point, pointRight; // 左右目相机坐标系下的三维点坐标
    Vector2d uv, uvRight;       // 左右目的像素坐标（归一化坐标）
    Vector2d velocity, velocityRight; // 左右目的像素速度（光流）
    bool is_stereo;             // 是否为双目观测的标志
};

// FeaturePerId类：表示一个特征点在多帧中的完整跟踪信息
class FeaturePerId
{
  public:
    const int feature_id;       // 特征的唯一ID标识
    int start_frame;            // 特征首次出现的帧ID
    vector<FeaturePerFrame> feature_per_frame; // 该特征在每一帧中的观测数据
    int used_num;               // 该特征被用于优化的次数（或观测到的次数）
    double estimated_depth;     // 特征的估计深度值（如果是单目）
    int solve_flag;             // 三角化求解状态：0-未求解，1-成功，2-失败

    // 构造函数：初始化特征跟踪
    // 参数：
    //   _feature_id: 特征唯一ID
    //   _start_frame: 特征首次出现的帧索引
    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
        // 初始化列表已设置所有值
    }

    // 返回该特征最后一次出现的帧索引（声明，未实现）
    int endFrame();
};

// FeatureManager类：管理所有特征点的类，包含各种特征处理函数
class FeatureManager
{
  public:
    // 构造函数：初始化特征管理器
    // 参数：
    //   _Rs: 旋转矩阵数组指针，通常存储每帧相机到世界坐标系的旋转
    FeatureManager(Matrix3d _Rs[]);

    // 设置相机到IMU的外参旋转矩阵
    // 参数：
    //   _ric: 双目相机的旋转外参数组，ric[0]左目，ric[1]右目
    void setRic(Matrix3d _ric[]);
    
    // 清空所有状态
    void clearState();
    
    // 获取特征数量
    int getFeatureCount();
    
    // 添加新特征并检查视差，用于关键帧选择
    // 参数：
    //   frame_count: 当前帧索引
    //   image: 图像特征数据，map格式：特征ID -> 该特征在帧中的观测数据列表
    //   td: 时间延迟
    // 返回值：是否需要将当前帧作为关键帧（基于视差检查）
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);
    
    // 获取两帧之间的对应特征点对（用于三角化或优化）
    // 参数：
    //   frame_count_l: 左帧索引
    //   frame_count_r: 右帧索引
    // 返回值：对应特征点的三维点对列表（世界坐标系下）
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    
    // 根据优化结果更新特征深度（注释掉，可能已移除或重构）
    //void updateDepth(const VectorXd &x);
    
    // 设置特征的深度值
    // 参数：
    //   x: 深度值向量，每个特征对应一个深度值
    void setDepth(const VectorXd &x);
    
    // 移除三角化失败的特征
    void removeFailures();
    
    // 清除所有特征的深度估计
    void clearDepth();
    
    // 获取所有特征的深度向量
    VectorXd getDepthVector();
    
    // 对特征进行三角化（恢复三维位置）
    // 参数：
    //   frameCnt: 帧数量
    //   Ps: 每帧相机在世界坐标系下的位置
    //   Rs: 每帧相机在世界坐标系下的旋转
    //   tic: 相机到IMU的平移外参
    //   ric: 相机到IMU的旋转外参
    void triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    
    // 使用两帧观测三角化单个特征点
    // 参数：
    //   Pose0, Pose1: 两帧相机的投影矩阵 [R|t]
    //   point0, point1: 两帧中的特征像素坐标
    //   point_3d: 输出的三维点坐标
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                            Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    
    // 使用PnP方法初始化帧位姿
    // 参数：
    //   frameCnt: 当前帧索引
    //   Ps, Rs: 待初始化的位置和旋转数组
    //   tic, ric: 相机-IMU外参
    void initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    
    // 使用PnP求解相机位姿
    // 参数：
    //   R_initial, P_initial: 初始的旋转和平移（作为输入和输出）
    //   pts2D: 2D图像点
    //   pts3D: 对应的3D空间点
    // 返回值：PnP求解是否成功
    bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial, 
                            vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);
    
    // 移除旧帧并调整深度（用于滑动窗口边缘化）
    // 参数：
    //   marg_R, marg_P: 被边缘化帧的旋转和平移
    //   new_R, new_P: 新参考帧的旋转和平移
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    
    // 移除滑动窗口中最老的帧
    void removeBack();
    
    // 移除滑动窗口中最新的帧（特定情况使用）
    // 参数：
    //   frame_count: 要移除的帧索引
    void removeFront(int frame_count);
    
    // 移除异常值特征
    // 参数：
    //   outlierIndex: 异常值特征的ID集合
    void removeOutlier(set<int> &outlierIndex);
    
    // 公共成员变量
    list<FeaturePerId> feature;  // 所有特征点的列表（使用list便于插入删除）
    int last_track_num;          // 上一帧跟踪到的特征数量
    double last_average_parallax; // 上一帧的平均视差（用于关键帧选择）
    int new_feature_num;         // 新特征数量
    int long_track_num;          // 长轨迹特征数量（被多帧跟踪的特征）

  private:
    // 计算补偿后的视差（用于关键帧选择）
    // 参数：
    //   it_per_id: 特征ID引用
    //   frame_count: 当前帧索引
    // 返回值：补偿后的视差值
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    
    // 私有成员变量
    const Matrix3d *Rs;  // 指向旋转矩阵数组的指针（每帧相机到世界坐标系的旋转）
    Matrix3d ric[2];     // 相机到IMU的旋转外参，ric[0]左目，ric[1]右目
};

#endif