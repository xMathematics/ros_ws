/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "utility.h"

// 根据重力向量计算旋转矩阵（用于VIO系统的初始化）
// 功能：计算一个旋转矩阵，使得重力向量对齐到Z轴正方向，同时保持水平方向（Yaw角）的稳定性
// 参数：
//   g: 重力向量（在初始帧的IMU坐标系下测量得到）
// 返回值：旋转矩阵，将输入坐标系旋转到重力对齐的世界坐标系
Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;  // 待计算的旋转矩阵
    
    // Step 1: 归一化重力向量，得到单位向量
    // ng1: 归一化后的重力向量（在IMU坐标系下的方向）
    Eigen::Vector3d ng1 = g.normalized();
    
    // Step 2: 定义目标重力方向 - 期望重力在Z轴正方向（世界坐标系定义）
    // 在VIO中，通常定义世界坐标系的Z轴向上（或向下，这里使用向上）
    // ng2: 目标重力方向向量 [0, 0, 1.0]^T
    Eigen::Vector3d ng2{0, 0, 1.0};
    
    // Step 3: 计算将ng1旋转到ng2的旋转矩阵
    // Eigen::Quaterniond::FromTwoVectors: 计算使向量a旋转到向量b的四元数
    // 这里计算从当前重力方向(ng1)旋转到Z轴(ng2)的旋转
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    
    // Step 4: 提取当前旋转矩阵的偏航角(Yaw)
    // 由于FromTwoVectors得到的旋转不是唯一的，可能包含任意偏航角
    // R2ypr返回[yaw, pitch, roll]向量，取第一个元素就是偏航角
    double yaw = Utility::R2ypr(R0).x();
    
    // Step 5: 消除偏航角的影响，保持水平方向不变
    // 乘以绕Z轴反向旋转-yaw的旋转矩阵，抵消偏航角
    // 这样得到的旋转矩阵只消除俯仰(Pitch)和滚转(Roll)，保持偏航角为0
    // 这意味着：世界坐标系的X轴方向保持与原始IMU坐标系的X轴在水平面的投影方向一致
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    
    // 注释掉的代码：如果需要额外的固定旋转（例如传感器安装角度为-90度），可以使用
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    
    // Step 6: 返回最终旋转矩阵
    return R0;
}