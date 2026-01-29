/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "pose_local_parameterization.h"

// 实现PoseLocalParameterization类的Plus函数
// 作用：将局部参数delta应用到全局参数x上，得到更新后的参数x_plus_delta
bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    // 将输入参数x的前3个元素映射为Eigen向量，表示平移部分
    // _p: 当前位姿的平移向量 [tx, ty, tz]
    // x指针布局: [tx, ty, tz, qx, qy, qz, qw]
    //          前3个double: 平移，后4个double: 四元数
    Eigen::Map<const Eigen::Vector3d> _p(x);
    
    // 将输入参数x的后4个元素映射为Eigen四元数，表示旋转部分
    // _q: 当前位姿的旋转四元数 [qx, qy, qz, qw]
    // 注意：Eigen::Quaterniond构造函数期望参数顺序为(w, x, y, z)
    // 但这里映射的是(x, y, z, w)，因为输入存储顺序是(x, y, z, w)
    // 这需要在构造时注意内存布局匹配
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    // 将delta的前3个元素映射为Eigen向量，表示平移增量
    // dp: 平移增量 [dtx, dty, dtz]
    // delta指针布局: [dtx, dty, dtz, drx, dry, drz]
    //               前3个double: 平移增量，后3个double: 旋转向量
    Eigen::Map<const Eigen::Vector3d> dp(delta);

    // 将delta的后3个元素(旋转向量)转换为四元数增量
    // delta+3: 指向delta数组的第4个元素，即旋转向量部分
    // Utility::deltaQ: 将旋转向量(so(3)李代数)转换为四元数增量
    // 函数实现：exp(δ/2)，其中δ是旋转向量
    // 注意：旋转向量θ的范数表示旋转角度，方向表示旋转轴
    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    // 将输出数组x_plus_delta的前3个元素映射为Eigen向量
    // p: 更新后的平移向量，用于存放结果
    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    
    // 将输出数组x_plus_delta的后4个元素映射为Eigen四元数
    // q: 更新后的旋转四元数，用于存放结果
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    // 计算更新后的平移：直接相加
    // p = 原始平移 + 平移增量
    p = _p + dp;
    
    // 计算更新后的旋转：四元数乘法后归一化
    // _q * dq: 将增量旋转dq应用到当前旋转_q上（四元数乘法表示旋转组合）
    // normalized(): 归一化，保证四元数是单位四元数
    // 注意：由于浮点计算精度问题，相乘后可能需要重新归一化
    q = (_q * dq).normalized();

    // 返回true表示操作成功
    return true;
}

// 实现PoseLocalParameterization类的ComputeJacobian函数
// 作用：计算局部参数化的雅可比矩阵 ∂(x ⊕ δ)/∂δ | δ=0
// 注意：这个实现是简化的，假设了特定的参数顺序
bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    // 将jacobian指针映射为Eigen矩阵，7行6列，按行优先存储
    // 矩阵大小：7×6 (全局参数维度 × 局部参数维度)
    // 行优先存储：Eigen::RowMajor
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    
    // 设置矩阵的前6行（行索引0-5）为单位矩阵
    // 这对应：∂(p ⊕ dp)/∂dp = I (3×3) 和 ∂(q ⊕ dq)/∂dq的前3行
    // 注意：这实际上是简化的雅可比，假设：
    // 1. 平移部分的雅可比是3×3单位矩阵
    // 2. 旋转部分的雅可比是3×3单位矩阵（近似）
    // 但实际上四元数对旋转向量的雅可比应该是4×3矩阵
    j.topRows<6>().setIdentity();
    
    // 设置矩阵的最后1行（第7行）为零
    // 这是因为第7行对应四元数的w分量对局部参数的导数
    // 在δ=0处的简化雅可比中，这部分为0
    // 注意：这实际上是不准确的，完整的雅可比应该是：
    // [ I_{3×3}      0_{3×3}   ]  // 平移对平移增量的导数
    // [ 0_{4×3}      J_{q/δ}(0) ]  // 四元数对旋转向量的导数
    j.bottomRows<1>().setZero();

    // 返回true表示计算成功
    return true;
}
