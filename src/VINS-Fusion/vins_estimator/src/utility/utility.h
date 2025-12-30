/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <cmath>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>

// Utility类：数学工具类，包含各种常用的数学变换和工具函数
// 所有函数都是静态的，可以直接通过类名调用
class Utility
{
  public:
    // 将旋转向量（轴角）转换为四元数（用于小角度近似）
    // 参数：
    //   theta: 旋转向量（3维），方向表示旋转轴，模长表示旋转角度
    // 返回值：对应的四元数
    // 原理：当旋转角度很小时，四元数可以近似为 [1, theta/2]
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;  // 提取模板的标量类型

        Eigen::Quaternion<Scalar_t> dq;  // 创建四元数对象
        // 将旋转向量除以2（因为四元数的虚部是旋转向量的一半）
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        
        // 设置四元数的实部为1（小角度近似）
        dq.w() = static_cast<Scalar_t>(1.0);
        // 设置四元数的虚部为half_theta
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }

    // 计算向量的反对称矩阵（叉乘矩阵）
    // 参数：
    //   q: 3维向量
    // 返回值：3x3反对称矩阵 [q]×，满足 [q]× * v = q × v
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;  // 创建3x3矩阵
        // 反对称矩阵的定义：
        // [0  -z   y]
        // [z   0  -x]
        // [-y  x   0]
        ans << typename Derived::Scalar(0), -q(2), q(1),
               q(2), typename Derived::Scalar(0), -q(0),
               -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }

    // 确保四元数的实部为正（四元数的双重覆盖性质：q和-q表示相同的旋转）
    // 参数：
    //   q: 输入四元数
    // 返回值：实部为正的四元数（或者原四元数）
    // 注意：实际函数体被注释掉了，目前直接返回原四元数
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
    {
        // 调试输出（已注释）
        //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
        //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
        //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
        
        // 如果实部小于0，返回-q；否则返回q
        //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : 
        //       Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
        return q;  // 目前直接返回原四元数
    }

    // 计算四元数左乘矩阵（用于四元数乘法：q * p）
    // 参数：
    //   q: 四元数
    // 返回值：4x4矩阵，左乘矩阵Q_l(q)，满足 vec(q * p) = Q_l(q) * vec(p)
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
    {
        // 确保四元数实部为正（双重覆盖处理）
        Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;  // 4x4矩阵
        
        // 构建左乘矩阵：
        // [w   -v^T]
        // [v   wI + [v]×]
        // 其中v是四元数的虚部向量
        ans(0, 0) = qq.w();                              // (0,0) = w
        ans.template block<1, 3>(0, 1) = -qq.vec().transpose();  // 第一行后三列 = -v^T
        ans.template block<3, 1>(1, 0) = qq.vec();                // 第一列后三行 = v
        // 右下3x3块 = w*I + [v]×
        ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() 
                                       + skewSymmetric(qq.vec());
        return ans;
    }

    // 计算四元数右乘矩阵（用于四元数乘法：p * q）
    // 参数：
    //   p: 四元数
    // 返回值：4x4矩阵，右乘矩阵Q_r(p)，满足 vec(p * q) = Q_r(p) * vec(q)
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
    {
        // 确保四元数实部为正
        Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        
        // 构建右乘矩阵：
        // [w   -v^T]
        // [v   wI - [v]×]
        // 注意：右乘矩阵与左乘矩阵的区别是右下角的符号
        ans(0, 0) = pp.w();                              // (0,0) = w
        ans.template block<1, 3>(0, 1) = -pp.vec().transpose();  // 第一行后三列 = -v^T
        ans.template block<3, 1>(1, 0) = pp.vec();                // 第一列后三行 = v
        // 右下3x3块 = w*I - [v]×
        ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() 
                                       - skewSymmetric(pp.vec());
        return ans;
    }

    // 将旋转矩阵转换为欧拉角（Yaw-Pitch-Roll，Z-Y-X顺序）
    // 参数：
    //   R: 3x3旋转矩阵
    // 返回值：欧拉角向量 [yaw, pitch, roll]（单位：度）
    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        // 提取旋转矩阵的三个列向量
        // n: 第一列，对应X轴在新坐标系下的方向（通常对应前进方向）
        // o: 第二列，对应Y轴在新坐标系下的方向
        // a: 第三列，对应Z轴在新坐标系下的方向（通常对应天向）
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);  // 存储欧拉角的向量
        
        // 计算偏航角（yaw）：绕Z轴旋转的角度
        // atan2(y, x) 计算点(x,y)与X轴正方向的夹角
        double y = atan2(n(1), n(0));
        
        // 计算俯仰角（pitch）：绕Y轴旋转的角度
        // 公式：pitch = atan2(-z, x*cos(yaw) + y*sin(yaw))
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        
        // 计算滚转角（roll）：绕X轴旋转的角度
        // 公式：roll = atan2(sin(yaw)*a_x - cos(yaw)*a_y, -sin(yaw)*o_x + cos(yaw)*o_y)
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), 
                        -o(0) * sin(y) + o(1) * cos(y));
        
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        // 将弧度转换为角度
        return ypr / M_PI * 180.0;
    }

    // 将欧拉角转换为旋转矩阵（Z-Y-X顺序）
    // 参数：
    //   ypr: 欧拉角向量 [yaw, pitch, roll]（单位：度）
    // 返回值：3x3旋转矩阵
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;  // 提取标量类型

        // 将角度转换为弧度
        Scalar_t y = ypr(0) / 180.0 * M_PI;  // 偏航角
        Scalar_t p = ypr(1) / 180.0 * M_PI;  // 俯仰角
        Scalar_t r = ypr(2) / 180.0 * M_PI;  // 滚转角

        // 绕Z轴旋转的旋转矩阵（偏航）
        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
              sin(y),  cos(y), 0,
              0,       0,      1;

        // 绕Y轴旋转的旋转矩阵（俯仰）
        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p),  0., sin(p),
              0.,      1., 0.,
              -sin(p), 0., cos(p);

        // 绕X轴旋转的旋转矩阵（滚转）
        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0.,      0.,
              0., cos(r), -sin(r),
              0., sin(r),  cos(r);

        // Z-Y-X顺序的旋转矩阵：R = Rz * Ry * Rx
        return Rz * Ry * Rx;
    }

    // 根据重力向量计算旋转矩阵（用于初始化）
    // 参数：
    //   g: 重力向量（在某个坐标系下测量）
    // 返回值：将重力对齐到Z轴的旋转矩阵
    static Eigen::Matrix3d g2R(const Eigen::Vector3d &g);  // 只有声明，实现在别处

    // 编译时循环展开的辅助模板元编程结构
    // 用于在编译时展开固定次数的循环，提高性能
    
    // 模板元编程的基础结构：uint_<N>
    template <size_t N>
    struct uint_
    {
    };

    // 递归展开函数：展开N次循环
    // 参数：
    //   f: 要执行的函数对象（lambda或函数指针）
    //   iter: 迭代器
    //   uint_<N>: 用于模板推导的辅助类型
    template <size_t N, typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<N>)
    {
        // 递归调用，展开N-1次
        unroller(f, iter, uint_<N - 1>());
        // 执行第N次迭代
        f(iter + N);
    }

    // 递归终止函数：展开0次循环（基本情况）
    template <typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<0>)
    {
        f(iter);  // 执行第0次迭代
    }

    // 角度归一化：将角度归一化到[-180, 180)度之间
    // 参数：
    //   angle_degrees: 输入角度（度）
    // 返回值：归一化后的角度
    template <typename T>
    static T normalizeAngle(const T& angle_degrees) {
      T two_pi(2.0 * 180);  // 360度
      
      // 如果角度大于0，减去360的整数倍
      if (angle_degrees > 0)
          return angle_degrees - two_pi * std::floor((angle_degrees + T(180)) / two_pi);
      else
          // 如果角度小于等于0，加上360的整数倍
          return angle_degrees + two_pi * std::floor((-angle_degrees + T(180)) / two_pi);
    };
};