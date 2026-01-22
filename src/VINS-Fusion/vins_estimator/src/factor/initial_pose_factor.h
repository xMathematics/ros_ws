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

// 继承自Ceres的SizedCostFunction，模板参数<6,7>表示：
// - 6：残差维度（3维平移误差 + 3维旋转误差）
// - 7：参数块维度（3维平移 + 4维四元数旋转）
class InitialPoseFactor : public ceres::SizedCostFunction<6, 7>
{
public:
    // 构造函数：接收初始位姿作为约束
    InitialPoseFactor(const Eigen::Vector3d &_P, const Eigen::Quaterniond &_Q)
    {
        // 存储初始位置
        init_P = _P;
        // 存储初始旋转（四元数形式）
        init_Q = _Q;
        // 信息矩阵的平方根，用于加权残差
        // 这里乘以1000表示这个约束很强，优化结果应该尽量靠近初始值
        sqrt_info = 1000 * Eigen::Matrix<double, 6, 6>::Identity();
    }
	// Ceres优化时调用的函数，计算残差和雅可比矩阵
	virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
	{
		// 1. 从优化变量中提取当前位姿
		// parameters[0] 是指向第一个参数块（7维位姿）的指针
		// 前3个是平移：x, y, z
		Eigen::Vector3d P(parameters[0][0], parameters[0][1], parameters[0][2]);
		
		// 后4个是四元数：注意构造函数参数顺序是(w, x, y, z)
		// 但存储顺序可能是(x, y, z, w)，这里需要根据实际存储调整
		Eigen::Quaterniond Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
		// 参数解释：
		// parameters[0][6] - 四元数实部 w
		// parameters[0][3] - 四元数虚部 x
		// parameters[0][4] - 四元数虚部 y  
		// parameters[0][5] - 四元数虚部 z

		// 2. 将residuals指针映射为Eigen向量，方便操作
		Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
		
		// 3. 计算平移部分的残差：当前平移 - 初始平移
		// 这部分很简单，就是向量差
		residual.block<3, 1>(0, 0) = P - init_P;
		
		// 4. 计算旋转部分的残差
		// 使用四元数差值来表示旋转误差
		// init_Q.inverse() * Q 计算从初始旋转到当前旋转的相对旋转
		// .vec() 取四元数的虚部（向量部分）
		// 乘以2是因为四元数的小角度近似：当旋转很小时，2 * vec(q) ≈ 旋转向量
		residual.block<3, 1>(3, 0) = 2 * (init_Q.inverse() * Q).vec();
		
		// 5. 应用信息矩阵加权
		// sqrt_info 是信息矩阵Λ的平方根，Λ = Σ^{-1}（协方差逆矩阵）
		// 加权后残差变为：r' = sqrt_info * r
		residual = sqrt_info * residual;

		// 6. 计算雅可比矩阵（如果请求）
		if (jacobians)
		{
			if (jacobians[0])  // 检查是否请求第一个参数块的雅可比
			{
				// 将jacobians[0]映射为6x7的Eigen矩阵（行优先存储）
				Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
				jacobian_pose.setZero();  // 初始化为零
				
				// 7. 平移部分的雅可比：∂(P - init_P)/∂P = I（单位矩阵）
				jacobian_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
				
				// 8. 旋转部分的雅可比
				// 计算 ∂(2*vec(init_Q^{-1} * Q))/∂δθ
				// 其中δθ是李代数扰动
				// Utility::Qleft(q) 返回四元数左乘矩阵（4x4）
				// .bottomRightCorner<3, 3>() 取右下角3x3部分
				// 这部分对应于旋转残差对旋转参数的导数
				jacobian_pose.block<3, 3>(3, 3) = Utility::Qleft(init_Q.inverse() * Q).bottomRightCorner<3, 3>();
				
				// 9. 应用信息矩阵加权到雅可比
				jacobian_pose = sqrt_info * jacobian_pose;
			}
		}
		return true;
	}

	// 调试函数：通过数值微分验证解析雅可比是否正确
	void check(double **parameters)
	{
		// 1. 分配内存存储残差和雅可比
		double *res = new double[6];
		double **jaco = new double *[1];
		jaco[0] = new double[6 * 7];  // 6x7的雅可比矩阵
		
		// 2. 调用Evaluate计算解析结果
		Evaluate(parameters, res, jaco);
		puts("check begins");
		
		// 3. 打印解析计算的残差和雅可比
		puts("my");  // "my"表示解析方法
		std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(res).transpose() << std::endl
				<< std::endl;
		std::cout << Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>>(jaco[0]) << std::endl
				<< std::endl;
		
		// 4. 重新计算残差用于对比
		Eigen::Vector3d P(parameters[0][0], parameters[0][1], parameters[0][2]);
		Eigen::Quaterniond Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
		
		Eigen::Matrix<double, 6, 1> residual;
		residual.block<3, 1>(0, 0) = P - init_P;
		residual.block<3, 1>(3, 0) = 2 * (init_Q.inverse() * Q).vec();
		residual = sqrt_info * residual;
		
		// 5. 打印数值计算的残差（应该与解析结果一致）
		puts("num");  // "num"表示数值方法
		std::cout << residual.transpose() << std::endl;
		
		// 6. 数值微分计算雅可比
		const double eps = 1e-6;  // 扰动大小
		Eigen::Matrix<double, 6, 6> num_jacobian;  // 数值雅可比（6x6）
		
		// 遍历6个自由度：前3个平移，后3个旋转
		for (int k = 0; k < 6; k++)
		{
			// 重新获取原始参数
			Eigen::Vector3d P(parameters[0][0], parameters[0][1], parameters[0][2]);
			Eigen::Quaterniond Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
			
			int a = k / 3;  // 0-平移，1-旋转
			int b = k % 3;  // x/y/z分量
			
			// 创建扰动向量
			Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;
			
			// 对平移或旋转施加扰动
			if (a == 0)
				P += delta;  // 平移扰动
			else if (a == 1)
				Q = Q * Utility::deltaQ(delta);  // 旋转扰动
			
			// 计算扰动后的残差
			Eigen::Matrix<double, 6, 1> tmp_residual;
			tmp_residual.block<3, 1>(0, 0) = P - init_P;
			tmp_residual.block<3, 1>(3, 0) = 2 * (init_Q.inverse() * Q).vec();
			tmp_residual = sqrt_info * tmp_residual;
			
			// 数值微分公式：∂r/∂x ≈ (r(x+Δx) - r(x)) / Δx
			num_jacobian.col(k) = (tmp_residual - residual) / eps;
		}
		
		// 7. 打印数值雅可比
		std::cout << num_jacobian << std::endl;
		
		// 8. 清理内存（原代码缺少delete，这里补充说明）
		// delete[] res;
		// delete[] jaco[0];
		// delete[] jaco;
	}

    Eigen::Vector3d init_P;
    Eigen::Quaterniond init_Q;
    Eigen::Matrix<double, 6, 6> sqrt_info;
};
