/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../estimator/parameters.h"

// 定义投影因子类，继承自Ceres库的SizedCostFunction
// 模板参数说明：
// <2, 7, 7, 7, 1> 表示：
// 2: 残差维度（2D像素坐标误差）
// 7: 第1个参数块维度（相机i的位姿：4D四元数+3D平移）
// 7: 第2个参数块维度（相机j的位姿：4D四元数+3D平移）
// 7: 第3个参数块维度（相机与IMU外参：4D四元数+3D平移）
// 1: 第4个参数块维度（特征点的逆深度）
class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>
{
  public:
    // 构造函数：初始化特征点在两个相机帧中的归一化坐标
    // 参数说明：
    // _pts_i: 特征点在相机i坐标系下的归一化坐标 [x_i, y_i, 1]^T
    // _pts_j: 特征点在相机j坐标系下的归一化坐标 [x_j, y_j, 1]^T
    // 注意：这是去畸变后的归一化平面坐标（z=1）
    ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    
    /**
     * @brief Ceres库要求的代价函数评估接口
     * 
     * @param parameters 输入参数数组，包含4个参数块
     *         parameters[0]: 相机i的位姿 [tx, ty, tz, qx, qy, qz, qw]
     *                        前3个为平移，后4个为四元数（Eigen顺序x,y,z,w）
     *         parameters[1]: 相机j的位姿 [tx, ty, tz, qx, qy, qz, qw]
     *         parameters[2]: 相机到IMU的外参 [tx, ty, tz, qx, qy, qz, qw]
     *         parameters[3]: 特征点的逆深度 [1/z]，标量
     * 
     * @param residuals 输出：计算得到的残差，维度2
     *                  [u_j_observed - u_j_predicted, 
     *                   v_j_observed - v_j_predicted]
     * 
     * @param jacobians 输出：雅可比矩阵数组，可选的
     *                  jacobians[0]: 残差对相机i位姿的雅可比，2×7矩阵
     *                  jacobians[1]: 残差对相机j位姿的雅可比，2×7矩阵
     *                  jacobians[2]: 残差对外参的雅可比，2×7矩阵
     *                  jacobians[3]: 残差对逆深度的雅可比，2×1矩阵
     *                  如果为nullptr，表示不需要计算对应雅可比
     * 
     * @return bool 评估是否成功
     */
    virtual bool Evaluate(double const *const *parameters, 
                         double *residuals, 
                         double **jacobians) const;
    
    // 调试函数：检查参数和计算过程，用于验证数值稳定性
    void check(double **parameters);
    
    // 成员变量
    
    // 特征点在两个相机帧的归一化坐标
    // pts_i: 在相机i坐标系下的归一化坐标 [x_i, y_i, 1]^T
    // pts_j: 在相机j坐标系下的归一化坐标 [x_j, y_j, 1]^T
    Eigen::Vector3d pts_i, pts_j;
    
    // 正切空间基矩阵，用于将3D坐标投影到2D残差空间
    // 维度2×3，作用：将三维点坐标（去中心化后）投影到二维图像平面
    // 通常定义为：tangent_base = [1, 0, -x; 0, 1, -y]^T
    // 用于计算重投影误差的切平面投影
    Eigen::Matrix<double, 2, 3> tangent_base;
    
    // 静态成员变量：信息矩阵的平方根
    // 用于对残差进行白化处理，考虑噪声特性
    // sqrt_info = sqrt(Σ^{-1})，其中Σ是观测噪声的协方差矩阵
    // 残差加权：r_whitened = sqrt_info * r_raw
    static Eigen::Matrix2d sqrt_info;
    
    // 静态成员变量：总时间统计，用于性能分析或调试
    static double sum_t;
};