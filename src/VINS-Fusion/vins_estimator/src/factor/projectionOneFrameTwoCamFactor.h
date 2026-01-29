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

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../estimator/parameters.h"

// 定义了一个Ceres代价函数类，继承自SizedCostFunction
// 模板参数说明：
// <2, 7, 7, 1, 1> 分别表示：
// 2 - 残差维度（这里是2D像素坐标的重投影误差）
// 7 - 第一个参数块大小（第一个相机的位姿，四元数+平移向量）
// 7 - 第二个参数块大小（第二个相机的位姿，四元数+平移向量）
// 1 - 第三个参数块大小（第一个相机的时间偏移）
// 1 - 第四个参数块大小（第二个相机的时间偏移）
class ProjectionOneFrameTwoCamFactor : public ceres::SizedCostFunction<2, 7, 7, 1, 1>
{
  public:
    // 构造函数
    // 参数说明：
    // _pts_i - 第一个相机坐标系下的3D点坐标（归一化平面坐标或像素坐标的齐次表示）
    // _pts_j - 第二个相机坐标系下的3D点坐标
    // _velocity_i - 第一个相机中该点的像素速度（用于时间偏移补偿）
    // _velocity_j - 第二个相机中该点的像素速度
    // _td_i - 第一个相机的时间偏移（用于滚动快门校正）
    // _td_j - 第二个相机的时间偏移
    ProjectionOneFrameTwoCamFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
    				   			   const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
    	   			   			   const double _td_i, const double _td_j);
    
    // 重写Ceres的Evaluate函数，计算残差和雅可比矩阵
    // 参数说明：
    // parameters - 参数数组，包含4个参数块：
    //   parameters[0] - 第一个相机的位姿 [tx, ty, tz, qx, qy, qz, qw] (平移在前，四元数在后)
    //   parameters[1] - 第二个相机的位姿
    //   parameters[2] - 第一个相机的时间偏移
    //   parameters[3] - 第二个相机的时间偏移
    // residuals - 输出参数，存储计算得到的残差（2维）
    // jacobians - 输出参数数组，存储各个参数块的雅可比矩阵
    //   如果jacobians[i]不为nullptr，则需要计算第i个参数块的雅可比
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    
    // 调试函数，检查参数是否正确（通常用于数值导数验证）
    void check(double **parameters);

    // 成员变量说明：
    Eigen::Vector3d pts_i, pts_j;          // 两个相机观测到的3D点坐标
    Eigen::Vector3d velocity_i, velocity_j; // 两个相机的像素速度（用于时间偏移补偿）
    double td_i, td_j;                     // 两个相机的时间偏移量

    // 切平面基向量，用于将3D点投影到切平面（通常用于计算重投影误差时降低维度）
    // 维度：2x3，将3维空间点投影到2维切平面
    Eigen::Matrix<double, 2, 3> tangent_base;

    // 静态成员变量：
    static Eigen::Matrix2d sqrt_info;  // 信息矩阵的平方根，用于对残差进行加权
                                       // 通常与观测噪声的协方差矩阵相关
    static double sum_t;               // 统计变量，用于记录总时间或其他统计信息
};