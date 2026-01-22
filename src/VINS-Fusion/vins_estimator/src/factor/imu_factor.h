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
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../estimator/parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

// IMU预积分因子类，继承自Ceres的SizedCostFunction
// 模板参数说明：
// <15, 7, 9, 7, 9> 表示：
// 15 - 残差维度（位置、旋转、速度、加速度计偏置、陀螺仪偏置各3维，共15维）
// 7  - 第1个参数块维度（姿态：平移3维 + 四元数4维）
// 9  - 第2个参数块维度（速度3维 + 加速度计偏置3维 + 陀螺仪偏置3维）
// 7  - 第3个参数块维度（姿态）
// 9  - 第4个参数块维度（速度、偏置）
class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
  public:
    // 删除默认构造函数，必须提供预积分对象
    IMUFactor() = delete;
    
    // 构造函数，接收预积分对象指针
    // 参数：
    // _pre_integration - 预积分对象指针，包含两帧之间的IMU测量积分结果
    IMUFactor(IntegrationBase* _pre_integration):pre_integration(_pre_integration)
    {
    }
    
    // Ceres代价函数的核心评估函数
    // 参数：
    // parameters - 二维数组，包含所有参数块的指针
    // residuals  - 输出残差向量
    // jacobians  - 输出雅可比矩阵数组（可选，可为nullptr）
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        // 从第1个参数块（索引0）提取第i帧的位姿
        // 参数格式：[px, py, pz, qx, qy, qz, qw]（注意：四元数存储顺序）
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);  // 位置
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);  // 四元数（w,x,y,z）

        // 从第2个参数块（索引1）提取第i帧的速度和偏置
        // 参数格式：[vx, vy, vz, bax, bay, baz, bgx, bgy, bgz]
        Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);      // 速度
        Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);    // 加速度计偏置
        Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);    // 陀螺仪偏置

        // 从第3个参数块（索引2）提取第j帧的位姿
        Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        // 从第4个参数块（索引3）提取第j帧的速度和偏置
        Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
        Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
        Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

        // 以下是注释掉的代码，展示IMU预积分的基本公式（用于参考理解）：
        // 预积分模型：
        // 预测的j帧状态 = i帧状态 ⊕ 预积分量
        // pPj = Pi + Vi * Δt - 0.5*g*Δt² + 修正后的位置预积分量
        // pQj = Qi * 旋转预积分量
        // pVj = Vi - g*Δt + 修正后的速度预积分量
        // pBaj = Bai (假设偏置随机游走)
        // pBgj = Bgi
        
        // 残差定义：
        // 位置残差：预测位置 - 实际位置
        // 旋转残差：预测旋转 - 实际旋转
        // 速度残差：预测速度 - 实际速度
        // 偏置残差：预测偏置 - 实际偏置
        
        // 预积分量计算公式：
        // Δp = Qi⁻¹ * (0.5*g*Δt² + Pj - Pi)  [忽略速度项]
        // Δv = Qi⁻¹ * (g*Δt + Vj - Vi)
        // Δq = Qi⁻¹ * Qj

        // 注释掉的偏置变化检测和重新传播（实际代码中被禁用）
        // 如果偏置变化超过阈值，重新进行预积分传播
        #if 0
        if ((Bai - pre_integration->linearized_ba).norm() > 0.10 ||
            (Bgi - pre_integration->linearized_bg).norm() > 0.01)
        {
            pre_integration->repropagate(Bai, Bgi);
        }
        #endif

        // 计算残差：将Eigen矩阵映射到residuals数组
        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
        // 调用预积分对象的evaluate方法计算15维残差
        residual = pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi,
                                            Pj, Qj, Vj, Baj, Bgj);

        // 计算信息矩阵的平方根（用于白化残差，考虑IMU噪声）
        // 首先计算协方差矩阵的逆的Cholesky分解，得到下三角矩阵L
        // L.transpose() * L = 协方差矩阵的逆（即信息矩阵）
        Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();
        // 如果不需要信息矩阵，可以设置为单位矩阵（调试用）
        // sqrt_info.setIdentity();
        
        // 白化残差：将残差乘以信息矩阵的平方根
        // 这相当于考虑IMU测量的不确定性，使优化问题变成马氏距离最小化
        residual = sqrt_info * residual;

        // 如果请求计算雅可比矩阵（Ceres优化需要）
        if (jacobians)
        {
            // 获取预积分时间间隔
            double sum_dt = pre_integration->sum_dt;
            
            // 从预积分雅可比矩阵中提取各个子块
            // 预积分对偏置的雅可比矩阵，用于修正预积分量（一阶近似）
            // 这些雅可比矩阵是在线性化点处计算的
            Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(O_P, O_BA);  // 位置对加速度计偏置的导数
            Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(O_P, O_BG);  // 位置对陀螺仪偏置的导数
            
            Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(O_R, O_BG);  // 旋转对陀螺仪偏置的导数
            
            Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(O_V, O_BA);  // 速度对加速度计偏置的导数
            Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(O_V, O_BG);  // 速度对陀螺仪偏置的导数

            // 数值稳定性检查：如果雅可比矩阵元素过大，可能数值不稳定
            if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8)
            {
                ROS_WARN("numerical unstable in preintegration");
                // 可选：打印雅可比矩阵或中断程序
            }

            // 计算第1个参数块（第i帧位姿）的雅可比矩阵
            if (jacobians[0])
            {
                // 将Ceres提供的原始数组映射为Eigen矩阵（行优先存储）
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();  // 初始化为零

                // 位置残差对第i帧位置的导数 = -Ri⁻¹
                jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
                
                // 位置残差对第i帧旋转的导数（使用李代数扰动模型）
                // 涉及旋转对平移的影响，使用反对称矩阵
                jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

                // 旋转残差对第i帧旋转的导数
                #if 0
                // 简化版本：不考虑偏置修正
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
                #else
                // 完整版本：考虑陀螺仪偏置修正
                // 计算修正后的旋转预积分量（考虑偏置变化）
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                // 使用四元数左乘和右乘的雅可比矩阵形式
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
                #endif

                // 速度残差对第i帧旋转的导数
                jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

                // 白化雅可比矩阵：乘以信息矩阵的平方根
                jacobian_pose_i = sqrt_info * jacobian_pose_i;

                // 数值稳定性检查
                if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in preintegration");
                }
            }
            
            // 计算第2个参数块（第i帧速度和偏置）的雅可比矩阵
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
                jacobian_speedbias_i.setZero();
                
                // 位置残差对第i帧速度的导数 = -Ri⁻¹ * Δt
                jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
                // 位置残差对第i帧加速度计偏置的导数 = -预积分的位置对加速度计偏置雅可比
                jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
                // 位置残差对第i帧陀螺仪偏置的导数 = -预积分的位置对陀螺仪偏置雅可比
                jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

                // 旋转残差对第i帧陀螺仪偏置的导数
                #if 0
                // 简化版本
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
                #else
                // 完整版本：考虑偏置修正链式法则
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * pre_integration->delta_q).bottomRightCorner<3, 3>() * dq_dbg;
                #endif

                // 速度残差对第i帧速度的导数 = -Ri⁻¹
                jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
                // 速度残差对第i帧加速度计偏置的导数
                jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
                // 速度残差对第i帧陀螺仪偏置的导数
                jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

                // 加速度计偏置残差对第i帧加速度计偏置的导数 = -I
                jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();
                // 陀螺仪偏置残差对第i帧陀螺仪偏置的导数 = -I
                jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

                // 白化雅可比矩阵
                jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;
            }
            
            // 计算第3个参数块（第j帧位姿）的雅可比矩阵
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
                jacobian_pose_j.setZero();

                // 位置残差对第j帧位置的导数 = Ri⁻¹
                jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

                // 旋转残差对第j帧旋转的导数
                #if 0
                // 简化版本
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
                #else
                // 完整版本：考虑偏置修正
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
                #endif

                // 白化雅可比矩阵
                jacobian_pose_j = sqrt_info * jacobian_pose_j;
            }
            
            // 计算第4个参数块（第j帧速度和偏置）的雅可比矩阵
            if (jacobians[3])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
                jacobian_speedbias_j.setZero();

                // 速度残差对第j帧速度的导数 = Ri⁻¹
                jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

                // 加速度计偏置残差对第j帧加速度计偏置的导数 = I
                jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

                // 陀螺仪偏置残差对第j帧陀螺仪偏置的导数 = I
                jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

                // 白化雅可比矩阵
                jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;
            }
        }

        return true;  // 成功完成评估
    }

    // 成员变量：预积分对象指针
    IntegrationBase* pre_integration;
    
    // 注意：以下成员函数被注释掉了，可能是调试或备用函数
    // bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);
    // void checkCorrection();
    // void checkTransition();
    // void checkJacobian(double **parameters);

};