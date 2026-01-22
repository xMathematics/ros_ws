/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "marginalization_factor.h"

// 类方法：计算残差块的信息（包括残差、雅可比矩阵，并应用损失函数）
void ResidualBlockInfo::Evaluate()
{
    // 调整残差向量大小以匹配代价函数的残差维度
    residuals.resize(cost_function->num_residuals());

    // 获取代价函数中每个参数块的大小
    std::vector<int> block_sizes = cost_function->parameter_block_sizes();
    
    // 分配原始雅可比矩阵指针数组，用于传递给代价函数
    raw_jacobians = new double *[block_sizes.size()];
    
    // 调整雅可比矩阵向量大小以匹配参数块数量
    jacobians.resize(block_sizes.size());

    // 遍历每个参数块，准备雅可比矩阵存储空间
    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
    {
        // 为每个雅可比矩阵分配内存：行数=残差维度，列数=参数块维度
        jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
        
        // 将Eigen矩阵的数据指针存储到原始指针数组中
        raw_jacobians[i] = jacobians[i].data();
        
        // 注释掉的代码：用于计算总维度（考虑李代数的情况）
        //dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
    }
    
    // 调用代价函数的Evaluate方法计算残差和雅可比矩阵
    // 参数：
    //   parameter_blocks.data() - 参数块指针数组
    //   residuals.data() - 残差输出数组
    //   raw_jacobians - 雅可比矩阵输出数组
    cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);

    // 注释掉的代码块：原本用于计算和检查信息矩阵的特征值
    // 这部分代码被注释掉，可能是调试或验证用的

    // 如果存在损失函数，应用鲁棒核函数
    if (loss_function)
    {
        // 定义缩放因子和alpha平方范数
        double residual_scaling_, alpha_sq_norm_;

        // 定义平方范数和损失函数计算的三个rho值
        double sq_norm, rho[3];

        // 计算残差的平方范数
        sq_norm = residuals.squaredNorm();
        
        // 调用损失函数计算rho值
        loss_function->Evaluate(sq_norm, rho);
        //printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm, rho[0], rho[1], rho[2]);

        // 计算rho[1]的平方根
        double sqrt_rho1_ = sqrt(rho[1]);

        // 处理特殊情况：残差为0或rho[2]非正（二次近似无效）
        if ((sq_norm == 0.0) || (rho[2] <= 0.0))
        {
            // 使用一阶近似
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_ = 0.0;
        }
        else
        {
            // 计算二阶修正项
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }

        // 对每个参数块的雅可比矩阵应用鲁棒核函数的缩放和修正
        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
        {
            // 应用公式：J' = √ρ₁ * (J - (α/||r||²) * r * (rᵀJ))
            jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
        }

        // 缩放残差向量
        residuals *= residual_scaling_;
    }
}

// 类析构函数：清理边缘化信息
MarginalizationInfo::~MarginalizationInfo()
{
    //ROS_WARN("release marginlizationinfo");
    
    // 清理参数块数据
    for (auto it = parameter_block_data.begin(); it != parameter_block_data.end(); ++it)
        delete it->second;  // 删除动态分配的数据

    // 清理因子（残差块信息）
    for (int i = 0; i < (int)factors.size(); i++)
    {
        // 删除原始雅可比矩阵指针数组
        delete[] factors[i]->raw_jacobians;
        
        // 删除代价函数对象
        delete factors[i]->cost_function;

        // 删除残差块信息对象本身
        delete factors[i];
    }
}

// 函数：添加残差块信息到边缘化信息中
// 参数：
//   residual_block_info - 指向残差块信息的指针，包含代价函数、参数块、要边缘化的参数集等信息
void MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo *residual_block_info)
{
    // 将残差块信息添加到factors向量中
    factors.emplace_back(residual_block_info);

    // 获取残差块信息中的参数块地址向量
    std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;
    // 获取代价函数中每个参数块的大小
    std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();

    // 遍历所有参数块，记录每个参数块地址对应的大小
    for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++)
    {
        // 获取参数块的内存地址
        double *addr = parameter_blocks[i];
        // 获取参数块的大小
        int size = parameter_block_sizes[i];
        // 将地址（转换为long类型作为键）和大小存储到映射中
        parameter_block_size[reinterpret_cast<long>(addr)] = size;
    }

    // 遍历要边缘化（丢弃）的参数块集合
    for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++)
    {
        // 获取要丢弃的参数块地址（通过drop_set中的索引找到对应的参数块地址）
        double *addr = parameter_blocks[residual_block_info->drop_set[i]];
        // 将地址和初始索引0存储到映射中，标记这些参数块将被边缘化
        parameter_block_idx[reinterpret_cast<long>(addr)] = 0;
    }
}

// 函数：预边缘化处理，计算所有残差块的信息并备份参数数据
void MarginalizationInfo::preMarginalize()
{
    // 遍历所有的残差块信息
    for (auto it : factors)
    {
        // 计算当前残差块的残差和雅可比矩阵
        it->Evaluate();

        // 获取当前残差块代价函数的参数块大小
        std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();
        // 遍历当前残差块的所有参数块
        for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
        {
            // 获取参数块的内存地址（转换为long类型作为键）
            long addr = reinterpret_cast<long>(it->parameter_blocks[i]);
            // 获取参数块的大小
            int size = block_sizes[i];
            // 如果该参数块的数据还没有备份过
            if (parameter_block_data.find(addr) == parameter_block_data.end())
            {
                // 为参数块数据分配内存
                double *data = new double[size];
                // 将原始参数块数据复制到新分配的内存中
                memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
                // 将备份的数据存储到映射中，键为参数块地址
                parameter_block_data[addr] = data;
            }
        }
    }
}

// 函数：计算参数块的局部维度大小（用于优化）
// 参数：
//   size - 原始参数块大小
// 返回值：
//   如果原始大小为7（位姿使用四元数+平移向量表示），返回6（李代数维度）
//   否则返回原始大小
int MarginalizationInfo::localSize(int size) const
{
    return size == 7 ? 6 : size;
}

// 函数：计算参数块的全局维度大小（用于存储）
// 参数：
//   size - 局部参数块大小
// 返回值：
//   如果局部大小为6（李代数维度），返回7（四元数+平移向量表示）
//   否则返回原始大小
int MarginalizationInfo::globalSize(int size) const
{
    return size == 6 ? 7 : size;
}

// 函数：线程函数，用于并行构建海森矩阵A和向量b
// 参数：
//   threadsstruct - 指向线程结构体的指针，包含子因子、参数块索引、参数块大小等信息
// 返回值：
//   返回传入的线程结构体指针
void* ThreadsConstructA(void* threadsstruct)
{
    // 将void*指针转换为ThreadsStruct*类型
    ThreadsStruct* p = ((ThreadsStruct*)threadsstruct);
    
    // 遍历该线程分配的所有子残差块（因子）
    for (auto it : p->sub_factors)
    {
        // 遍历当前残差块的所有参数块（对每个参数块i）
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
        {
            // 获取参数块i在边缘化矩阵中的起始索引
            int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
            // 获取参数块i的原始大小
            int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
            // 如果参数块大小为7（位姿），则使用6维的李代数表示
            if (size_i == 7)
                size_i = 6;
            // 获取参数块i的雅可比矩阵，只取前size_i列（对应于局部参数化）
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
            
            // 遍历参数块j（从i开始，只计算上三角部分，利用对称性）
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                // 获取参数块j在边缘化矩阵中的起始索引
                int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                // 获取参数块j的原始大小
                int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
                // 如果参数块大小为7（位姿），则使用6维的李代数表示
                if (size_j == 7)
                    size_j = 6;
                // 获取参数块j的雅可比矩阵，只取前size_j列
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                
                // 计算海森矩阵块：J_i^T * J_j
                if (i == j)
                {
                    // 对角线块：直接累加
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                }
                else
                {
                    // 非对角线块：计算上三角部分
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    // 下三角部分通过转置得到，利用对称性
                    p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            // 计算向量b的对应部分：J_i^T * residuals
            p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    return threadsstruct;
}

// 函数：执行边缘化操作，构建先验约束
void MarginalizationInfo::marginalize()
{
    // 第一步：为要边缘化的参数块分配索引
    int pos = 0;
    // 遍历parameter_block_idx（包含要边缘化的参数块）
    for (auto &it : parameter_block_idx)
    {
        // 为每个要边缘化的参数块分配连续的索引
        it.second = pos;
        // 更新索引位置，使用局部维度（如果是位姿则使用6维李代数）
        pos += localSize(parameter_block_size[it.first]);
    }

    // m: 要边缘化的参数块的总维度（将被移除的状态量维度）
    m = pos;

    // 第二步：为剩余（保留）的参数块分配索引
    for (const auto &it : parameter_block_size)
    {
        // 检查该参数块是否不在要边缘化的集合中（即要保留的参数块）
        if (parameter_block_idx.find(it.first) == parameter_block_idx.end())
        {
            // 为保留的参数块分配索引，从m位置开始
            parameter_block_idx[it.first] = pos;
            // 更新索引位置
            pos += localSize(it.second);
        }
    }

    // n: 保留的参数块的总维度（剩余状态量维度）
    n = pos - m;
    // 打印调试信息
    //ROS_INFO("marginalization, pos: %d, m: %d, n: %d, size: %d", pos, m, n, (int)parameter_block_idx.size());
    
    // 如果m=0，表示没有要边缘化的参数块，边缘化无效
    if(m == 0)
    {
        valid = false;
        printf("unstable tracking...\n");
        return;
    }

    // 第三步：构建整个问题的海森矩阵A和向量b（高斯牛顿方程：A = J^T J, b = J^T r）
    TicToc t_summing;  // 计时器
    Eigen::MatrixXd A(pos, pos);  // 海森矩阵，大小为(pos, pos)
    Eigen::VectorXd b(pos);       // 向量b，大小为pos
    A.setZero();  // 初始化A为零矩阵
    b.setZero();  // 初始化b为零向量
    
    // 注释掉的单线程版本代码，用于构建A和b
    /*
    for (auto it : factors)
    {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
        {
            int idx_i = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
            int size_i = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])]);
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                int idx_j = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                int size_j = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])]);
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                if (i == j)
                    A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                else
                {
                    A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    A.block(idx_j, idx_i, size_j, size_i) = A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    ROS_INFO("summing up costs %f ms", t_summing.toc());
    */
    
    // 第四步：多线程构建海森矩阵A和向量b
    TicToc t_thread_summing;  // 多线程计时器
    
    // 创建线程ID数组和线程数据结构数组
    pthread_t tids[NUM_THREADS];
    ThreadsStruct threadsstruct[NUM_THREADS];
    
    // 将因子（残差块）均匀分配到各个线程
    int i = 0;
    for (auto it : factors)
    {
        threadsstruct[i].sub_factors.push_back(it);
        i++;
        i = i % NUM_THREADS;  // 循环分配
    }
    
    // 创建并启动线程
    for (int i = 0; i < NUM_THREADS; i++)
    {
        TicToc zero_matrix;  // 计时器
        // 初始化线程的A和b为零
        threadsstruct[i].A = Eigen::MatrixXd::Zero(pos,pos);
        threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
        // 传递参数块大小和索引映射
        threadsstruct[i].parameter_block_size = parameter_block_size;
        threadsstruct[i].parameter_block_idx = parameter_block_idx;
        
        // 创建线程，执行ThreadsConstructA函数
        int ret = pthread_create( &tids[i], NULL, ThreadsConstructA ,(void*)&(threadsstruct[i]));
        if (ret != 0)
        {
            ROS_WARN("pthread_create error");
            ROS_BREAK();  // 发生错误时中断程序
        }
    }
    
    // 等待所有线程完成，并合并结果
    for( int i = NUM_THREADS - 1; i >= 0; i--)  
    {
        pthread_join( tids[i], NULL );  // 等待线程结束
        // 累加各线程计算的A和b
        A += threadsstruct[i].A;
        b += threadsstruct[i].b;
    }
    //ROS_DEBUG("thread summing up costs %f ms", t_thread_summing.toc());
    //ROS_INFO("A diff %f , b diff %f ", (A - tmp_A).sum(), (b - tmp_b).sum());

    // 第五步：执行舒尔补（Schur Complement）操作，边缘化掉m个变量
    
    // 首先确保Amm是对称的（取0.5*(Amm + Amm^T)）
    Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
    
    // 对Amm进行特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
    // 断言最小特征值大于等于-1e-4（确保正定性）
    //ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue %f", saes.eigenvalues().minCoeff());

    // 计算Amm的伪逆（处理奇异值）
    // 对大于eps的特征值取倒数，小于等于eps的设为0
    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * 
                              Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() * 
                              saes.eigenvectors().transpose();
    //printf("error1: %f\n", (Amm * Amm_inv - Eigen::MatrixXd::Identity(m, m)).sum());

    // 提取子矩阵和子向量
    Eigen::VectorXd bmm = b.segment(0, m);           // 要边缘化变量对应的b部分
    Eigen::MatrixXd Amr = A.block(0, m, m, n);       // A的右上部分（m×n）
    Eigen::MatrixXd Arm = A.block(m, 0, n, m);       // A的左下部分（n×m）
    Eigen::MatrixXd Arr = A.block(m, m, n, n);       // A的右下部分（n×n）
    Eigen::VectorXd brr = b.segment(m, n);           // 保留变量对应的b部分
    
    // 舒尔补操作：计算新的A和b（先验信息）
    // A_new = Arr - Arm * Amm_inv * Amr
    // b_new = brr - Arm * Amm_inv * bmm
    A = Arr - Arm * Amm_inv * Amr;
    b = brr - Arm * Amm_inv * bmm;

    // 第六步：对新的先验信息进行分解，得到雅可比矩阵和残差
    
    // 对新的A进行特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
    
    // 计算特征值的平方根和平方根倒数（用于构建雅可比和残差）
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));
    
    // 计算特征值的平方根和平方根倒数
    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
    
    // 构建线性化后的雅可比矩阵和残差
    // 这样可以将先验信息表示为：||J_prior * x + r_prior||^2
    linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
    
    // 调试输出
    //std::cout << A << std::endl << std::endl;
    //std::cout << linearized_jacobians << std::endl;
    //printf("error2: %f %f\n", (linearized_jacobians.transpose() * linearized_jacobians - A).sum(),
    //      (linearized_jacobians.transpose() * linearized_residuals - b).sum());
}

// 函数：获取边缘化后保留的参数块信息
// 参数：
//   addr_shift - 地址偏移映射表，存储参数块旧地址到新地址的映射
// 返回值：
//   返回保留参数块的新地址向量
std::vector<double *> MarginalizationInfo::getParameterBlocks(std::unordered_map<long, double *> &addr_shift)
{
    // 清空保留参数块的相关信息，准备重新填充
    keep_block_addr.clear();
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();

    // 遍历所有参数块的索引映射
    for (const auto &it : parameter_block_idx)
    {
        // 检查参数块索引是否大于等于m（即是否是要保留的参数块）
        // m是之前计算的要边缘化的参数块总维度
        if (it.second >= m)
        {
            // 添加参数块的大小
            keep_block_size.push_back(parameter_block_size[it.first]);
            // 添加参数块在边缘化矩阵中的全局索引
            keep_block_idx.push_back(parameter_block_idx[it.first]);
            // 添加参数块的备份数据（来自preMarginalize）
            keep_block_data.push_back(parameter_block_data[it.first]);
            // 添加参数块的新地址（通过addr_shift映射）
            keep_block_addr.push_back(addr_shift[it.first]);
        }
    }
    
    // 计算所有保留参数块的总维度大小
    sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);

    // 返回保留参数块的新地址向量
    return keep_block_addr;
}

// 类构造函数：边缘化因子
// 参数：
//   _marginalization_info - 指向边缘化信息的指针
MarginalizationFactor::MarginalizationFactor(MarginalizationInfo* _marginalization_info):marginalization_info(_marginalization_info)
{
    int cnt = 0;
    // 遍历保留参数块的大小，添加到参数块大小列表中
    for (auto it : marginalization_info->keep_block_size)
    {
        // 将参数块大小添加到Ceres优化框架的参数块大小列表中
        mutable_parameter_block_sizes()->push_back(it);
        cnt += it;  // 累加总维度
    }
    // 打印调试信息
    //printf("residual size: %d, %d\n", cnt, n);
    
    // 设置残差维度为边缘化信息中的n（保留参数块的总维度）
    set_num_residuals(marginalization_info->n);
};

// 函数：计算边缘化因子的残差和雅可比矩阵
// 参数：
//   parameters - 参数块指针数组，包含所有保留参数块的当前值
//   residuals  - 残差输出数组
//   jacobians  - 雅可比矩阵输出数组（可选的，如果为nullptr则不计算雅可比）
// 返回值：
//   总是返回true，表示计算成功
bool MarginalizationFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    // 调试信息，打印内部地址
    //printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(), num_residuals());
    //for (int i = 0; i < static_cast<int>(keep_block_size.size()); i++)
    //{
    //    //printf("unsigned %x\n", reinterpret_cast<unsigned long>(parameters[i]));
    //    //printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
    //printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
    //printf("residual %x\n", reinterpret_cast<long>(residuals));
    //}
    
    // 获取边缘化信息中的维度信息
    int n = marginalization_info->n;  // 保留参数块的总维度
    int m = marginalization_info->m;  // 被边缘化的参数块的总维度
    
    // 定义状态增量向量dx，大小为n
    Eigen::VectorXd dx(n);
    
    // 遍历所有保留参数块，计算状态增量dx
    for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
    {
        // 获取当前参数块的大小
        int size = marginalization_info->keep_block_size[i];
        // 计算当前参数块在dx中的起始索引（减去m，因为m之前的索引对应被边缘化的参数块）
        int idx = marginalization_info->keep_block_idx[i] - m;
        
        // 将当前参数值映射为Eigen向量
        Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
        // 将参数块的线性化点（备份数据）映射为Eigen向量
        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size);
        
        // 根据参数块类型计算状态增量
        if (size != 7)
        {
            // 对于非位姿参数块（如速度、偏置等），直接计算差值
            dx.segment(idx, size) = x - x0;
        }
        else
        {
            // 对于位姿参数块（3维平移 + 4维四元数）
            // 平移部分：直接计算差值
            dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
            
            // 旋转部分：使用四元数的李代数扰动
            // 计算相对旋转：q0^{-1} * q
            Eigen::Quaterniond q0_inv(x0(6), x0(3), x0(4), x0(5)).inverse();
            Eigen::Quaterniond q(x(6), x(3), x(4), x(5));
            Eigen::Quaterniond dq = q0_inv * q;
            
            // 使用positify确保四元数的实部非负（保持唯一表示）
            dq = Utility::positify(dq);
            
            // 计算旋转的李代数扰动（乘以2，因为四元数到旋转向量的映射关系）
            dx.segment<3>(idx + 3) = 2.0 * dq.vec();
            
            // 检查处理后的四元数实部是否非负，如果不是则取反
            if (!(dq.w() >= 0))
            {
                dx.segment<3>(idx + 3) = 2.0 * -Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * 
                                                                  Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            }
        }
    }
    
    // 计算残差：r = r0 + J * dx
    // 其中r0是线性化残差，J是线性化雅可比矩阵
    Eigen::Map<Eigen::VectorXd>(residuals, n) = marginalization_info->linearized_residuals + 
                                                 marginalization_info->linearized_jacobians * dx;
    
    // 如果请求计算雅可比矩阵
    if (jacobians)
    {
        // 遍历所有保留参数块
        for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
        {
            // 检查当前参数块的雅可比矩阵指针是否存在
            if (jacobians[i])
            {
                // 获取参数块大小和局部维度大小
                int size = marginalization_info->keep_block_size[i];
                int local_size = marginalization_info->localSize(size);  // 对于位姿，7->6
                
                // 计算当前参数块在dx中的起始索引
                int idx = marginalization_info->keep_block_idx[i] - m;
                
                // 将Ceres的雅可比矩阵指针映射为Eigen矩阵（按行主序）
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> 
                    jacobian(jacobians[i], n, size);
                
                // 初始化雅可比矩阵为零
                jacobian.setZero();
                
                // 将线性化雅可比矩阵的对应列复制到当前参数块的雅可比矩阵中
                // 注意：对于位姿参数块，只使用前6列（局部参数化）
                jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);
            }
        }
    }
    
    return true;
}