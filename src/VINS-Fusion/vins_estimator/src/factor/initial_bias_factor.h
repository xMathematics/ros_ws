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

// 定义初始偏置因子类，继承自Ceres的固定尺寸成本函数
// 模板参数<6, 9>表示：残差维度为6，参数块维度为9
class InitialBiasFactor : public ceres::SizedCostFunction<6, 9>
{
  public:
    // 构造函数，接收初始加速度计偏置和陀螺仪偏置
    InitialBiasFactor(const Eigen::Vector3d &_Ba, const Eigen::Vector3d &_Bg)
    {
        // 保存传入的初始偏置值
    	init_Ba = _Ba;  // 初始加速度计偏置
    	init_Bg = _Bg;  // 初始陀螺仪偏置
        
        // 计算信息矩阵的平方根（用于加权残差）
        // 1.0/0.001 = 1000，相当于噪声标准差为0.001的倒数
        // 创建6x6的单位矩阵并乘以1000
    	sqrt_info = 1.0 / (0.001) * Eigen::Matrix<double, 6, 6>::Identity();
    }
    
    // 重写Evaluate函数，计算残差和雅可比矩阵
    // parameters: 参数数组，每个元素指向一个参数块
    // residuals: 输出残差数组
    // jacobians: 输出雅可比矩阵数组（可选）
    virtual bool Evaluate(double const *const *parameters, 
                         double *residuals, 
                         double **jacobians) const
    {
        // 从参数块中提取当前的偏置估计值
        // parameters[0]指向9维参数块，其中：
        // [0-2]: 可能其他参数（如旋转等），此处未使用
        // [3-5]: 加速度计偏置Ba
        // [6-8]: 陀螺仪偏置Bg
    	Eigen::Vector3d Ba(parameters[0][3], parameters[0][4], parameters[0][5]);
    	Eigen::Vector3d Bg(parameters[0][6], parameters[0][7], parameters[0][8]);

        // 将residuals指针映射为Eigen向量，便于操作
    	Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        
        // 计算残差：当前估计值与初始值的差
        // 前3维：加速度计偏置残差
    	residual.block<3, 1>(0, 0) = Ba - init_Ba;
        // 后3维：陀螺仪偏置残差
    	residual.block<3, 1>(3, 0) = Bg - init_Bg;
        
        // 用信息矩阵的平方根加权残差（相当于马氏距离）
    	residual = sqrt_info * residual;

        // 如果请求计算雅可比矩阵
    	if (jacobians)
    	{
            // 如果请求第一个参数块（也是唯一一个）的雅可比
    		if (jacobians[0])
    		{
                // 将jacobians[0]映射为6x9的矩阵（按行主序）
    		    Eigen::Map<Eigen::Matrix<double, 6, 9, Eigen::RowMajor>> jacobian_bias(jacobians[0]);
                
                // 雅可比矩阵初始化为零
    		    jacobian_bias.setZero();
                
                // 设置非零部分：
                // 残差前3行（对应Ba残差）对参数第3-5列（对应Ba参数）的导数为单位矩阵
    		    jacobian_bias.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
                // 残差后3行（对应Bg残差）对参数第6-8列（对应Bg参数）的导数为单位矩阵
    		    jacobian_bias.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
                
                // 用信息矩阵加权雅可比
    		    jacobian_bias = sqrt_info * jacobian_bias;
    		}
    	}
    	return true;  // 计算成功
    }

    // 成员变量
    Eigen::Vector3d init_Ba, init_Bg;  // 保存的初始偏置值
    Eigen::Matrix<double, 6, 6> sqrt_info;  // 信息矩阵的平方根（用于加权）
};