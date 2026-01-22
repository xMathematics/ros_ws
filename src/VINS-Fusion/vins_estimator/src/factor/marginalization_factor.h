/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <pthread.h>
#include <ceres/ceres.h>
#include <unordered_map>

#include "../utility/utility.h"
#include "../utility/tic_toc.h"

// 定义线程数，用于并行处理边缘化计算
const int NUM_THREADS = 4;

// 残差块信息结构体：存储一个残差块的所有信息
struct ResidualBlockInfo
{
    // 构造函数
    // 参数：
    // _cost_function - Ceres代价函数指针
    // _loss_function - Ceres损失函数指针（用于鲁棒核函数）
    // _parameter_blocks - 参数块指针数组
    // _drop_set - 要边缘化掉的参数块索引集合
    ResidualBlockInfo(ceres::CostFunction *_cost_function, ceres::LossFunction *_loss_function, 
                      std::vector<double *> _parameter_blocks, std::vector<int> _drop_set)
        : cost_function(_cost_function), loss_function(_loss_function), 
          parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}

    // 评估函数：计算残差和雅可比矩阵
    void Evaluate();

    // 代价函数和损失函数
    ceres::CostFunction *cost_function;
    ceres::LossFunction *loss_function;
    
    // 参数块指针数组
    std::vector<double *> parameter_blocks;
    
    // 要边缘化掉的参数块索引（在parameter_blocks中的索引）
    std::vector<int> drop_set;

    // 原始雅可比矩阵数组（Ceres格式）
    double **raw_jacobians;
    
    // 存储雅可比矩阵的向量（每个参数块对应一个雅可比矩阵）
    // Eigen::RowMajor表示行优先存储，与Ceres默认格式一致
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    
    // 残差向量
    Eigen::VectorXd residuals;

    // 计算局部参数化后的维度（用于李群）
    // 对于姿态参数（维度7，四元数+平移），局部维度为6（李代数）
    // 其他参数（维度不为7）保持原维度
    int localSize(int size)
    {
        return size == 7 ? 6 : size;
    }
};

// 线程结构体：用于并行计算
struct ThreadsStruct
{
    std::vector<ResidualBlockInfo *> sub_factors;  // 分配给该线程的残差块
    Eigen::MatrixXd A;                             // 该线程计算的Hessian矩阵部分
    Eigen::VectorXd b;                             // 该线程计算的梯度向量部分
    
    // 参数块ID到全局维度的映射
    std::unordered_map<long, int> parameter_block_size; //global size
    
    // 参数块ID到局部索引的映射（在边缘化中的位置）
    std::unordered_map<long, int> parameter_block_idx; //local size
};

// 边缘化信息类：管理边缘化过程
class MarginalizationInfo
{
  public:
    // 构造函数
    MarginalizationInfo(){valid = true;};
    
    // 析构函数
    ~MarginalizationInfo();
    
    // 计算局部维度（考虑李群局部参数化）
    int localSize(int size) const;
    
    // 计算全局维度
    int globalSize(int size) const;
    
    // 添加残差块信息到边缘化器
    void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);
    
    // 预边缘化：计算所有残差块的雅可比矩阵和残差
    void preMarginalize();
    
    // 执行边缘化：舒尔补计算
    void marginalize();
    
    // 获取参数块的新地址（用于滑动窗口优化）
    std::vector<double *> getParameterBlocks(std::unordered_map<long, double *> &addr_shift);

    // 所有残差块信息
    std::vector<ResidualBlockInfo *> factors;
    
    // m: 要边缘化的参数维度，n: 要保留的参数维度
    int m, n;
    
    // 参数块ID到全局维度的映射
    std::unordered_map<long, int> parameter_block_size; //global size
    
    // 总参数块大小（所有要边缘化的参数维度总和）
    int sum_block_size;
    
    // 参数块ID到局部索引的映射
    std::unordered_map<long, int> parameter_block_idx; //local size
    
    // 参数块ID到数据指针的映射
    std::unordered_map<long, double *> parameter_block_data;

    // 要保留的参数块的全局维度
    std::vector<int> keep_block_size; //global size
    
    // 要保留的参数块在边缘化后的索引
    std::vector<int> keep_block_idx;  //local size
    
    // 要保留的参数块的数据指针
    std::vector<double *> keep_block_data;

    // 线性化点的雅可比矩阵
    Eigen::MatrixXd linearized_jacobians;
    
    // 线性化点的残差
    Eigen::VectorXd linearized_residuals;
    
    // 数值稳定性参数
    const double eps = 1e-8;
    
    // 有效性标志
    bool valid;
};

// 边缘化因子类：继承Ceres的CostFunction
// 用于将边缘化先验作为约束加入后续的优化问题
class MarginalizationFactor : public ceres::CostFunction
{
  public:
    // 构造函数：接收边缘化信息
    MarginalizationFactor(MarginalizationInfo* _marginalization_info);
    
    // Ceres代价函数评估函数
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    // 边缘化信息指针
    MarginalizationInfo* marginalization_info;
};
