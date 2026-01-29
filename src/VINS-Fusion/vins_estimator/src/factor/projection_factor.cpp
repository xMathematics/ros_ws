/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "projection_factor.h"

Eigen::Matrix2d ProjectionFactor::sqrt_info;
double ProjectionFactor::sum_t;

// 构造函数：初始化特征点在两个相机帧中的归一化坐标
ProjectionFactor::ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j) 
    : pts_i(_pts_i), pts_j(_pts_j)  // 初始化成员变量
{
#ifdef UNIT_SPHERE_ERROR
    // 如果使用单位球面误差模型（鱼眼/全景相机）
    Eigen::Vector3d b1, b2;  // 定义两个正交基向量
    Eigen::Vector3d a = pts_j.normalized();  // j帧特征点归一化
    Eigen::Vector3d tmp(0, 0, 1);  // 临时向量，z轴方向
    if(a == tmp)  // 如果a与z轴重合
        tmp << 1, 0, 0;  // 改为x轴方向
    // 计算第一个切平面基向量：从tmp减去a方向的分量并归一化
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    // 计算第二个切平面基向量：与a和b1正交
    b2 = a.cross(b1);
    // 构建切平面基矩阵（2x3），用于将3D向量投影到切平面
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();
#endif
};

// 计算残差和雅可比矩阵的主函数
bool ProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;  // 计时器
    
    // 从优化变量中提取第i帧的IMU位姿 [tx, ty, tz, qx, qy, qz, qw]
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    // 注意：四元数顺序为(w, x, y, z)
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    
    // 提取第j帧的IMU位姿
    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    
    // 提取相机到IMU的外参（平移和旋转）
    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    
    // 提取逆深度参数
    double inv_dep_i = parameters[3][0];
    
    // ------------------ 前向传播：计算重投影 ------------------
    // 将i帧的归一化坐标转换为相机坐标系下的3D点
    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    // 转换到i帧IMU坐标系
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    // 转换到世界坐标系
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    // 转换到j帧IMU坐标系
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    // 转换到j帧相机坐标系
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    
    // 映射残差数组到Eigen向量
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    
#ifdef UNIT_SPHERE_ERROR 
    // 单位球面误差：计算在切平面上的投影误差
    residual = tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
#else
    // 针孔相机误差模型：计算归一化平面上的重投影误差
    double dep_j = pts_camera_j.z();  // j帧相机坐标系下的深度
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
#endif
    
    // 应用信息矩阵（协方差的逆）对残差加权
    residual = sqrt_info * residual;
    
    // ------------------ 计算雅可比矩阵 ------------------
    if (jacobians)
    {
        // 将四元数转换为旋转矩阵
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        
        // reduce矩阵：误差对j帧相机坐标的导数
        Eigen::Matrix<double, 2, 3> reduce(2, 3);
        
#ifdef UNIT_SPHERE_ERROR
        // 单位球面误差的雅可比计算
        double norm = pts_camera_j.norm();  // 范数
        Eigen::Matrix3d norm_jaco;  // 归一化操作的雅可比矩阵
        double x1, x2, x3;
        x1 = pts_camera_j(0);
        x2 = pts_camera_j(1);
        x3 = pts_camera_j(2);
        // 归一化操作的雅可比：d(normalized(x))/dx
        norm_jaco << 1.0 / norm - x1 * x1 / pow(norm, 3), - x1 * x2 / pow(norm, 3), - x1 * x3 / pow(norm, 3),
                     - x1 * x2 / pow(norm, 3), 1.0 / norm - x2 * x2 / pow(norm, 3), - x2 * x3 / pow(norm, 3),
                     - x1 * x3 / pow(norm, 3), - x2 * x3 / pow(norm, 3), 1.0 / norm - x3 * x3 / pow(norm, 3);
        reduce = tangent_base * norm_jaco;
#else
        // 针孔模型误差的雅可比：d([x/z, y/z]^T)/d([x, y, z]^T)
        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
                  0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
#endif
        
        // 应用信息矩阵
        reduce = sqrt_info * reduce;
        
        // ------------------ 计算关于第i帧位姿的雅可比 ------------------
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            
            // jaco_i: d(pts_camera_j)/d(第i帧位姿的6自由度)
            Eigen::Matrix<double, 3, 6> jaco_i;
            // 平移部分：d(pts_camera_j)/d(Pi)
            jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
            // 旋转部分：d(pts_camera_j)/d(theta_i)，theta_i为旋转的李代数
            jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);
            
            // 链式法则：reduce * jaco_i
            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            // 最后一列为0（对时间戳的导数）
            jacobian_pose_i.rightCols<1>().setZero();
        }
        
        // ------------------ 计算关于第j帧位姿的雅可比 ------------------
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
            
            Eigen::Matrix<double, 3, 6> jaco_j;
            // 平移部分：d(pts_camera_j)/d(Pj)
            jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
            // 旋转部分：d(pts_camera_j)/d(theta_j)
            jaco_j.rightCols<3>() = ric.transpose() * Utility::skewSymmetric(pts_imu_j);
            
            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
        
        // ------------------ 计算关于外参的雅可比 ------------------
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            
            // 平移部分：d(pts_camera_j)/d(tic)
            jaco_ex.leftCols<3>() = ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
            
            // 旋转部分：d(pts_camera_j)/d(theta_ic)，theta_ic为外参旋转的李代数
            Eigen::Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
            jaco_ex.rightCols<3>() = -tmp_r * Utility::skewSymmetric(pts_camera_i) 
                                   + Utility::skewSymmetric(tmp_r * pts_camera_i)
                                   + Utility::skewSymmetric(ric.transpose() * (Rj.transpose() * (Ri * tic + Pi - Pj) - tic));
            
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        
        // ------------------ 计算关于逆深度的雅可比 ------------------
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
            
#if 1
            // 标准公式：d(pts_camera_j)/d(inv_dep_i)
            // pts_camera_i = pts_i / inv_dep_i，所以d(pts_camera_i)/d(inv_dep_i) = -pts_i / (inv_dep_i^2)
            jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
#else
            // 另一种写法（可能用于调试）
            jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i;
#endif
        }
    }
    
    // 累加计算时间
    sum_t += tic_toc.toc();
    
    return true;
}


// 雅可比矩阵数值检查函数
void ProjectionFactor::check(double **parameters)
{
    // 分配内存存储残差和雅可比矩阵
    double *res = new double[15];  // 残差数组（虽然实际只有2维，但分配15个以防万一）
    double **jaco = new double *[4];  // 雅可比矩阵指针数组，对应4个参数块
    
    // 为每个参数块分配雅可比矩阵内存
    jaco[0] = new double[2 * 7];  // 第i帧位姿：2×7（残差维度2，参数维度7）
    jaco[1] = new double[2 * 7];  // 第j帧位姿：2×7
    jaco[2] = new double[2 * 7];  // 外参：2×7
    jaco[3] = new double[2 * 1];  // 逆深度：2×1
    
    // 调用Evaluate函数计算解析的残差和雅可比矩阵
    Evaluate(parameters, res, jaco);
    
    // 输出开始检查的提示
    puts("check begins");
    puts("my");  // "my"可能表示解析方法的结果
    
    // 打印解析方法计算的残差（2维向量）
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 1>>(res).transpose() << std::endl << std::endl;
    
    // 打印第i帧位姿的解析雅可比矩阵（2×7）
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[0]) << std::endl << std::endl;
    
    // 打印第j帧位姿的解析雅可比矩阵（2×7）
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[1]) << std::endl << std::endl;
    
    // 打印外参的解析雅可比矩阵（2×7）
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[2]) << std::endl << std::endl;
    
    // 打印逆深度的解析雅可比矩阵（2×1）
    std::cout << Eigen::Map<Eigen::Vector2d>(jaco[3]) << std::endl << std::endl;
    
    // 从参数中提取各变量（与Evaluate函数中相同）
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    
    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    
    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    double inv_dep_i = parameters[3][0];
    
    // 重投影计算（与Evaluate函数中相同）
    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    
    // 计算残差
    Eigen::Vector2d residual;
#ifdef UNIT_SPHERE_ERROR 
    residual = tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
#else
    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
#endif
    residual = sqrt_info * residual;  // 应用信息矩阵
    
    // 打印数值方法计算的残差
    puts("num");  // "num"可能表示数值方法
    std::cout << residual.transpose() << std::endl;
    
    // 设置扰动步长（数值差分法的步长）
    const double eps = 1e-6;
    // 数值雅可比矩阵：2×19（残差维度2，总变量数19）
    Eigen::Matrix<double, 2, 19> num_jacobian;
    
    // 数值差分法计算雅可比矩阵：对每个变量进行扰动
    for (int k = 0; k < 19; k++)
    {
        // 每次循环开始时重新从原始参数提取变量
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        
        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
        
        Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
        double inv_dep_i = parameters[3][0];
        
        // 确定要扰动哪个变量
        int a = k / 3;  // 参数块索引：0~6（共7个参数块）
        int b = k % 3;  // 在参数块中的维度索引：0~2
        
        // 创建扰动向量delta：在对应维度上加eps
        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;
        
        // 根据参数块索引a应用扰动
        if (a == 0)          // 第i帧平移
            Pi += delta;
        else if (a == 1)     // 第i帧旋转（使用四元数扰动）
            Qi = Qi * Utility::deltaQ(delta);
        else if (a == 2)     // 第j帧平移
            Pj += delta;
        else if (a == 3)     // 第j帧旋转
            Qj = Qj * Utility::deltaQ(delta);
        else if (a == 4)     // 外参平移
            tic += delta;
        else if (a == 5)     // 外参旋转
            qic = qic * Utility::deltaQ(delta);
        else if (a == 6)     // 逆深度（只有一维，使用delta.x()）
            inv_dep_i += delta.x();
        
        // 使用扰动后的参数重新计算重投影
        Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
        Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
        Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
        Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
        Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
        
        // 计算扰动后的残差
        Eigen::Vector2d tmp_residual;
#ifdef UNIT_SPHERE_ERROR 
        tmp_residual = tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
#else
        double dep_j = pts_camera_j.z();
        tmp_residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
#endif
        tmp_residual = sqrt_info * tmp_residual;
        
        // 数值差分计算雅可比：d(residual)/d(variable) ≈ (f(x+Δx) - f(x)) / Δx
        num_jacobian.col(k) = (tmp_residual - residual) / eps;
    }
    
    // 打印数值方法计算的雅可比矩阵
    std::cout << num_jacobian << std::endl;
    
    // 注意：这里应该释放分配的内存，但代码中缺少delete语句
    // 实际使用时应添加：
    // delete[] res;
    // for(int i = 0; i < 4; i++) delete[] jaco[i];
    // delete[] jaco;
}