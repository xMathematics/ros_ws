/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

// 定义PoseLocalParameterization类，继承自ceres::LocalParameterization
// 用于处理3D位姿（旋转+平移）的局部参数化
// 全局参数维度为7（四元数[4] + 平移[3]），局部参数维度为6（旋转向量[3] + 平移[3]）
class PoseLocalParameterization : public ceres::LocalParameterization
{
    /**
     * @brief Plus操作：将局部参数delta加到全局参数x上
     * 
     * @param x 输入：当前全局参数，维度为GlobalSize()=7
     *         格式：[qx, qy, qz, qw, tx, ty, tz]
     *         qx,qy,qz,qw: 四元数表示旋转（实部在最后）
     *         tx,ty,tz: 平移向量
     * 
     * @param delta 输入：局部参数更新量，维度为LocalSize()=6
     *         格式：[d_rot_x, d_rot_y, d_rot_z, d_trans_x, d_trans_y, d_trans_z]
     *         d_rot_*: 旋转向量（so(3)李代数），表示旋转更新
     *         d_trans_*: 平移更新
     * 
     * @param x_plus_delta 输出：更新后的全局参数，维度为7
     * 
     * @return bool 操作是否成功
     * 
     * 功能：x_plus_delta = x ⊕ delta
     * 其中⊕表示在流形上的加法操作：
     * 1. 将旋转向量delta[0:3]转换为旋转矩阵增量
     * 2. 将增量应用到当前四元数上
     * 3. 平移部分直接相加：x[4:7] + delta[3:6]
     */
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    
    /**
     * @brief 计算局部参数化的雅可比矩阵
     * 
     * @param x 输入：当前全局参数，维度为7
     * 
     * @param jacobian 输出：雅可比矩阵，维度为7×6
     *         矩阵形式：∂x_plus_delta/∂delta 在delta=0处的值
     *         对应公式：J = ∂(x ⊕ delta)/∂delta | delta=0
     *         
     *         矩阵结构：
     *         [ ∂(旋转更新)/∂(旋转向量)    0_{4×3}     ]
     *         [   0_{3×3}               I_{3×3}     ]
     *         其中左上角4×3块是四元数对旋转向量的导数
     * 
     * @return bool 计算是否成功
     */
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    
    /**
     * @brief 返回全局参数的维度
     * @return int 全局参数维度：7
     *         对应：四元数(4) + 平移向量(3)
     */
    virtual int GlobalSize() const { return 7; };
    
    /**
     * @brief 返回局部参数的维度（正切空间维度）
     * @return int 局部参数维度：6
     *         对应：旋转向量(3) + 平移向量(3)
     *         注：旋转只有3个自由度，四元数4维但受单位约束
     */
    virtual int LocalSize() const { return 6; };
};