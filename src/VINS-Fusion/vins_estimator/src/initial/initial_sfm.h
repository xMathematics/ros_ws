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

#pragma once 
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
using namespace Eigen;
using namespace std;



// SFM特征点结构体，用于存储每个特征点在多帧中的信息
struct SFMFeature
{
    bool state;  // 特征点的状态（是否被成功三角化等）
    int id;  // 特征点的唯一标识符
    
    // 观测向量：每个元素是一个<帧ID, 像素坐标>对
    // 存储该特征点在不同帧中的观测位置
    vector<pair<int,Vector2d>> observation;
    
    double position[3];  // 特征点的3D世界坐标 [x, y, z]
    double depth;  // 特征点的深度值（相对于某个参考帧）
};

// 3D重投影误差结构体，用于Ceres优化框架
struct ReprojectionError3D
{
    // 构造函数：传入观测到的像素坐标(u,v)
    ReprojectionError3D(double observed_u, double observed_v)
        :observed_u(observed_u), observed_v(observed_v)  // 初始化列表
        {}
    
    // 重载()运算符：Ceres需要这个函数来计算残差
    // 模板参数T允许自动微分（可以是double或Jet类型）
    // 参数说明：
    // - camera_R: 相机旋转（四元数，4个元素）
    // - camera_T: 相机平移（3个元素）
    // - point: 3D点坐标（3个元素）
    // - residuals: 输出的残差（2个元素：[u误差, v误差]）
    template <typename T>
    bool operator()(const T* const camera_R, const T* const camera_T, const T* point, T* residuals) const
    {
        T p[3];  // 临时存储旋转后的点
        
        // 使用四元数旋转3D点：p = R * point
        ceres::QuaternionRotatePoint(camera_R, point, p);
        
        // 加上平移：p = R * point + T
        p[0] += camera_T[0];
        p[1] += camera_T[1];
        p[2] += camera_T[2];
        
        // 投影到归一化平面：xp = p[0]/p[2], yp = p[1]/p[2]
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];
        
        // 计算重投影误差：观测值 - 投影值
        residuals[0] = xp - T(observed_u);
        residuals[1] = yp - T(observed_v);
        
        return true;  // 总是返回true，除非有数值问题
    }
    
    // 静态工厂函数：创建Ceres代价函数
    // 参数说明：
    // - observed_x, observed_y: 观测到的像素坐标
    // 返回值：Ceres代价函数指针
    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y) 
    {
        // 使用自动微分创建代价函数：
        // 模板参数说明：
        // - ReprojectionError3D: 误差计算类
        // - 2: 残差维度（u,v两个误差）
        // - 4: 第一个参数块大小（旋转四元数）
        // - 3: 第二个参数块大小（平移向量）
        // - 3: 第三个参数块大小（3D点坐标）
        return (new ceres::AutoDiffCostFunction<
                ReprojectionError3D, 2, 4, 3, 3>(
                new ReprojectionError3D(observed_x, observed_y)));
    }
    
    double observed_u;  // 存储观测的u坐标
    double observed_v;  // 存储观测的v坐标
};

// 全局SFM类：实现完整的SFM流程
class GlobalSFM
{
public:
    // 构造函数
    GlobalSFM();
    
    // 主要的SFM构建函数
    // 参数说明：
    // - frame_num: 总帧数
    // - q: 相机旋转四元数数组（输入输出参数）
    // - T: 相机平移向量数组（输入输出参数）
    // - l: 参考帧（通常是第一帧）
    // - relative_R, relative_T: 相对位姿（通常是第l帧和第l+1帧之间的）
    // - sfm_f: SFM特征点数组
    // - sfm_tracked_points: 输出三角化后的3D点（特征点ID到3D坐标的映射）
    // 返回值：SFM是否成功
    bool construct(int frame_num, Quaterniond* q, Vector3d* T, int l,
                   const Matrix3d relative_R, const Vector3d relative_T,
                   vector<SFMFeature> &sfm_f, map<int, Vector3d> &sfm_tracked_points);

private:
    // 通过PnP求解相机位姿
    // 参数说明：
    // - R_initial, P_initial: 初始位姿（输入输出参数）
    // - i: 需要求解的帧索引
    // - sfm_f: SFM特征点数组
    // 返回值：PnP求解是否成功
    bool solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i, 
                         vector<SFMFeature> &sfm_f);
    
    // 三角化一个点
    // 参数说明：
    // - Pose0, Pose1: 两个相机的位姿矩阵[3x4]
    // - point0, point1: 两个视图中的对应点坐标
    // - point_3d: 输出的3D点坐标
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, 
                          Eigen::Matrix<double, 3, 4> &Pose1,
                          Vector2d &point0, Vector2d &point1, 
                          Vector3d &point_3d);
    
    // 三角化两帧之间的所有匹配点
    // 参数说明：
    // - frame0, frame1: 两帧的索引
    // - Pose0, Pose1: 两帧的相机位姿
    // - sfm_f: SFM特征点数组
    void triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0, 
                              int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
                              vector<SFMFeature> &sfm_f);
    
    int feature_num;  // 特征点数量（类成员变量，但在这段代码中未看到初始化）
};


