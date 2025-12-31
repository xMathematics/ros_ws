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

#include "initial_sfm.h"

// GlobalSFM类的构造函数
GlobalSFM::GlobalSFM()
// 构造函数体为空，没有执行任何初始化操作
{}
// 注意：这里使用了默认构造函数，通常用于初始化类的成员变量
// 但在这个实现中，feature_num等成员变量没有在构造函数中初始化
// 可能需要在其他地方初始化，或者使用初始化列表

// 三角化函数：从两视图几何中恢复3D点坐标
// 参数说明：
// - Pose0, Pose1: 两个相机的投影矩阵 [3x4] = K[R|t]
// - point0, point1: 在两个视图中的归一化坐标（或像素坐标，取决于内参）
// - point_3d: 输出的三角化后的3D点坐标（齐次坐标归一化后）
void GlobalSFM::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, 
                                 Eigen::Matrix<double, 3, 4> &Pose1,
                                 Vector2d &point0, Vector2d &point1, 
                                 Vector3d &point_3d)
{
    // 设计矩阵 A，大小为 4x4
    // 用于存储线性方程组 Ax=0 的系数矩阵
    Matrix4d design_matrix = Matrix4d::Zero();  // 初始化为零矩阵
    
    // 构建线性方程组：使用直接线性变换（DLT）方法
    
    // 第一行：来自第一个视图的x坐标约束
    // 对于归一化坐标(u,v)，有：u = (P0.row(0)·X)/(P0.row(2)·X)
    // 等价于：(u * P0.row(2) - P0.row(0))·X = 0
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    
    // 第二行：来自第一个视图的y坐标约束
    // v = (P0.row(1)·X)/(P0.row(2)·X)
    // 等价于：(v * P0.row(2) - P0.row(1))·X = 0
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    
    // 第三行：来自第二个视图的x坐标约束
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    
    // 第四行：来自第二个视图的y坐标约束
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    
    // 使用SVD求解齐次线性方程组 A * X = 0
    Vector4d triangulated_point;  // 存储齐次坐标下的3D点 [X,Y,Z,W]
    
    // SVD分解：A = UΣV^T，解是V的最后一列（最小奇异值对应的右奇异向量）
    triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV)
                         .matrixV()  // 获取右奇异向量矩阵V
                         .rightCols<1>();  // 取最后一列
    
    // 将齐次坐标转换为欧氏坐标：除以最后一个分量W
    point_3d(0) = triangulated_point(0) / triangulated_point(3);  // X = X/W
    point_3d(1) = triangulated_point(1) / triangulated_point(3);  // Y = Y/W
    point_3d(2) = triangulated_point(2) / triangulated_point(3);  // Z = Z/W
}


// PnP求解函数：通过已知的3D-2D点对应关系求解相机位姿
// 参数说明：
// - R_initial, P_initial: 初始的旋转矩阵和平移向量（输入输出参数）
// - i: 当前要求解的帧索引
// - sfm_f: 特征点数组
// 返回值：PnP求解是否成功
bool GlobalSFM::solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i,
                                vector<SFMFeature> &sfm_f)
{
    // 准备OpenCV需要的2D和3D点容器
    vector<cv::Point2f> pts_2_vector;  // 存储2D图像点
    vector<cv::Point3f> pts_3_vector;  // 存储对应的3D世界点
    
    // 遍历所有特征点
    for (int j = 0; j < feature_num; j++)
    {
        // 只处理已经成功三角化的特征点（state=true）
        if (sfm_f[j].state != true)
            continue;  // 跳过未三角化的点
        
        Vector2d point2d;  // 当前特征点在第i帧中的2D坐标
        
        // 遍历该特征点的所有观测记录
        for (int k = 0; k < (int)sfm_f[j].observation.size(); k++)
        {
            // 查找特征点在第i帧中的观测
            if (sfm_f[j].observation[k].first == i)
            {
                // 获取在第i帧中的图像坐标
                Vector2d img_pts = sfm_f[j].observation[k].second;
                
                // 将Eigen::Vector2d转换为OpenCV的Point2f
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);
                
                // 将3D世界点坐标转换为OpenCV的Point3f
                cv::Point3f pts_3(sfm_f[j].position[0], 
                                  sfm_f[j].position[1], 
                                  sfm_f[j].position[2]);
                pts_3_vector.push_back(pts_3);
                
                break;  // 找到对应观测后跳出内层循环
            }
        }
    }
    
    // 检查匹配点数量是否足够进行PnP求解
    if (int(pts_2_vector.size()) < 15)
    {
        printf("unstable features tracking, please slowly move you device!\n");
        // 如果匹配点少于10个，认为不可靠，返回失败
        if (int(pts_2_vector.size()) < 10)
            return false;
    }
    
    // OpenCV变量声明
    cv::Mat r, rvec, t, D, tmp_r;
    
    // 将Eigen旋转矩阵转换为OpenCV格式
    // 1. 将Eigen::Matrix3d转换为cv::Mat
    cv::eigen2cv(R_initial, tmp_r);
    
    // 2. 将旋转矩阵转换为旋转向量（罗德里格斯变换）
    // 因为OpenCV的solvePnP使用旋转向量表示旋转
    cv::Rodrigues(tmp_r, rvec);
    
    // 将Eigen平移向量转换为OpenCV格式
    cv::eigen2cv(P_initial, t);
    
    // 创建单位内参矩阵K
    // 这里假设使用归一化坐标，所以内参矩阵为单位矩阵
    // 如果使用像素坐标，应该使用真实的相机内参
    cv::Mat K = (cv::Mat_<double>(3, 3) << 
                 1, 0, 0,   // fx=1, cx=0
                 0, 1, 0,   // fy=1, cy=0
                 0, 0, 1);  // 齐次坐标
    
    bool pnp_succ;  // PnP求解成功标志
    
    // 使用OpenCV的solvePnP函数求解PnP问题
    // 参数说明：
    // - pts_3_vector: 3D世界点坐标
    // - pts_2_vector: 对应的2D图像点坐标
    // - K: 相机内参矩阵
    // - D: 畸变系数（这里为空）
    // - rvec: 旋转向量（输入初始值，输出结果）
    // - t: 平移向量（输入初始值，输出结果）
    // - 1: 使用迭代法（SOLVEPNP_ITERATIVE）
    pnp_succ = cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1);
    
    // 检查PnP求解是否成功
    if(!pnp_succ)
    {
        return false;  // 求解失败
    }
    
    // 将旋转向量转换回旋转矩阵
    cv::Rodrigues(rvec, r);
    
    // 将OpenCV结果转换回Eigen格式
    MatrixXd R_pnp;  // 旋转矩阵
    cv::cv2eigen(r, R_pnp);
    
    MatrixXd T_pnp;  // 平移向量
    cv::cv2eigen(t, T_pnp);
    
    // 更新输入的旋转和平移
    R_initial = R_pnp;
    P_initial = T_pnp;
    
    return true;  // 成功求解
}

// 三角化两帧之间的所有匹配特征点
// 参数说明：
// - frame0, frame1: 两帧的索引
// - Pose0, Pose1: 两帧的相机位姿矩阵 [3x4]
// - sfm_f: SFM特征点数组（输入输出参数）
void GlobalSFM::triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0, 
                                     int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
                                     vector<SFMFeature> &sfm_f)
{
    // 断言确保不是同一帧（不能自己和自己三角化）
    assert(frame0 != frame1);
    
    // 遍历所有特征点
    for (int j = 0; j < feature_num; j++)
    {
        // 如果特征点已经三角化成功，跳过
        if (sfm_f[j].state == true)
            continue;
        
        // 标志位：记录特征点是否在两帧中都有观测
        bool has_0 = false, has_1 = false;
        Vector2d point0;  // 在第frame0帧中的观测坐标
        Vector2d point1;  // 在第frame1帧中的观测坐标
        
        // 遍历该特征点的所有观测记录
        for (int k = 0; k < (int)sfm_f[j].observation.size(); k++)
        {
            // 查找在第frame0帧中的观测
            if (sfm_f[j].observation[k].first == frame0)
            {
                point0 = sfm_f[j].observation[k].second;  // 获取观测坐标
                has_0 = true;  // 标记已找到
            }
            // 查找在第frame1帧中的观测
            if (sfm_f[j].observation[k].first == frame1)
            {
                point1 = sfm_f[j].observation[k].second;  // 获取观测坐标
                has_1 = true;  // 标记已找到
            }
        }
        
        // 如果特征点在这两帧中都有观测，则进行三角化
        if (has_0 && has_1)
        {
            Vector3d point_3d;  // 存储三角化后的3D点坐标
            
            // 调用三角化函数
            triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
            
            // 更新特征点状态和位置
            sfm_f[j].state = true;  // 标记为已三角化
            sfm_f[j].position[0] = point_3d(0);  // X坐标
            sfm_f[j].position[1] = point_3d(1);  // Y坐标
            sfm_f[j].position[2] = point_3d(2);  // Z坐标
            
            // 可选：打印调试信息
            //cout << "trangulated : " << frame1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
        }							  
    }
}

// 全局SFM构建函数
// 参数说明：
// - frame_num: 总帧数
// - q: 世界坐标系到相机坐标系的旋转（四元数数组，输入输出参数）
// - T: 世界坐标系到相机坐标系的平移（向量数组，输入输出参数）
// - l: 参考帧索引（通常设为0，即第一帧）
// - relative_R: 相对旋转矩阵（第l帧到最后一帧）
// - relative_T: 相对平移向量（第l帧到最后一帧）
// - sfm_f: 特征点数组
// - sfm_tracked_points: 输出的三角化后的3D点（特征点ID到3D坐标的映射）
// 返回值：SFM是否成功构建
bool GlobalSFM::construct(int frame_num, Quaterniond* q, Vector3d* T, int l,
                          const Matrix3d relative_R, const Vector3d relative_T,
                          vector<SFMFeature> &sfm_f, map<int, Vector3d> &sfm_tracked_points)
{
    // 设置特征点数量
    feature_num = sfm_f.size();
    
    // 注释掉调试信息
    //cout << "set 0 and " << l << " as known " << endl;
    
    // 初始化：已知第l帧和最后一帧的位姿
    
    // 1. 设置第l帧的位姿（作为世界坐标系的原点）
    // 旋转四元数设置为单位四元数 [1, 0, 0, 0]
    q[l].w() = 1;
    q[l].x() = 0;
    q[l].y() = 0;
    q[l].z() = 0;
    // 平移向量设置为零向量
    T[l].setZero();
    
    // 2. 设置最后一帧的位姿（相对于第l帧）
    // 根据给定的相对位姿计算最后一帧的位姿
    q[frame_num - 1] = q[l] * Quaterniond(relative_R);  // 旋转：q_last = q_l * relative_q
    T[frame_num - 1] = relative_T;  // 平移（这里有问题，应该是经过旋转后的平移）
    
    // 注释掉调试信息
    //cout << "init q_l " << q[l].w() << " " << q[l].vec().transpose() << endl;
    //cout << "init t_l " << T[l].transpose() << endl;
    
    // 准备相机坐标系下的位姿
    
    // 定义各种格式的位姿存储：
    Matrix3d c_Rotation[frame_num];       // 相机坐标系下的旋转矩阵
    Vector3d c_Translation[frame_num];    // 相机坐标系下的平移向量
    Quaterniond c_Quat[frame_num];        // 相机坐标系下的旋转四元数
    double c_rotation[frame_num][4];      // C数组格式的旋转（未使用）
    double c_translation[frame_num][3];   // C数组格式的平移（未使用）
    Eigen::Matrix<double, 3, 4> Pose[frame_num];  // 相机投影矩阵 [R|t]
    
    // 计算第l帧在相机坐标系下的位姿
    c_Quat[l] = q[l].inverse();  // 相机到世界的逆 = 世界到相机
    c_Rotation[l] = c_Quat[l].toRotationMatrix();  // 转换为旋转矩阵
    c_Translation[l] = -1 * (c_Rotation[l] * T[l]);  // 平移：t_cam = -R * t_world
    // 构建投影矩阵 Pose = [R|t]
    Pose[l].block<3, 3>(0, 0) = c_Rotation[l];      // 旋转部分
    Pose[l].block<3, 1>(0, 3) = c_Translation[l];   // 平移部分
    
    // 计算最后一帧在相机坐标系下的位姿
    c_Quat[frame_num - 1] = q[frame_num - 1].inverse();
    c_Rotation[frame_num - 1] = c_Quat[frame_num - 1].toRotationMatrix();
    c_Translation[frame_num - 1] = -1 * (c_Rotation[frame_num - 1] * T[frame_num - 1]);
    Pose[frame_num - 1].block<3, 3>(0, 0) = c_Rotation[frame_num - 1];
    Pose[frame_num - 1].block<3, 1>(0, 3) = c_Translation[frame_num - 1];
    
    // SFM构建主流程：
    
    // 步骤1：三角化第l帧和最后一帧之间的特征点
    // 步骤2：从l+1帧到frame_num-2帧，依次执行：
    //        a. 使用PnP求解当前帧位姿
    //        b. 三角化当前帧与最后一帧之间的特征点
    for (int i = l; i < frame_num - 1 ; i++)
    {
        // 使用PnP求解第i帧位姿（除了第l帧，因为第l帧位姿已知）
        if (i > l)
        {
            // 使用前一帧的位姿作为初始值
            Matrix3d R_initial = c_Rotation[i - 1];
            Vector3d P_initial = c_Translation[i - 1];
            
            // 调用PnP求解
            if(!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
                return false;  // PnP求解失败，整个SFM失败
            
            // 更新当前帧的位姿
            c_Rotation[i] = R_initial;
            c_Translation[i] = P_initial;
            c_Quat[i] = c_Rotation[i];  // 从旋转矩阵构造四元数
            Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
            Pose[i].block<3, 1>(0, 3) = c_Translation[i];
        }
        
        // 三角化第i帧和最后一帧之间的特征点
        triangulateTwoFrames(i, Pose[i], frame_num - 1, Pose[frame_num - 1], sfm_f);
    }
    
    // 步骤3：三角化第l帧与l+1到frame_num-2帧之间的特征点
    // 增加三角化的基线，提高精度
    for (int i = l + 1; i < frame_num - 1; i++)
        triangulateTwoFrames(l, Pose[l], i, Pose[i], sfm_f);
    
    // 步骤4：处理第l帧之前的帧（从l-1到第0帧）
    //        a. 使用PnP求解当前帧位姿（使用后一帧作为初始值）
    //        b. 三角化当前帧与第l帧之间的特征点
    for (int i = l - 1; i >= 0; i--)
    {
        // 使用后一帧的位姿作为初始值（因为时间顺序上后一帧已经求解）
        Matrix3d R_initial = c_Rotation[i + 1];
        Vector3d P_initial = c_Translation[i + 1];
        
        // 调用PnP求解
        if(!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
            return false;
        
        // 更新当前帧的位姿
        c_Rotation[i] = R_initial;
        c_Translation[i] = P_initial;
        c_Quat[i] = c_Rotation[i];
        Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
        Pose[i].block<3, 1>(0, 3) = c_Translation[i];
        
        // 三角化第i帧和第l帧之间的特征点
        triangulateTwoFrames(i, Pose[i], l, Pose[l], sfm_f);
    }
    
    // 步骤5：三角化所有剩余的未三角化特征点
    // 对于每个特征点，使用其最早和最晚的观测帧进行三角化
    for (int j = 0; j < feature_num; j++)
    {
        // 跳过已经三角化的特征点
        if (sfm_f[j].state == true)
            continue;
        
        // 只有至少在两帧中观测到的特征点才能被三角化
        if ((int)sfm_f[j].observation.size() >= 2)
        {
            Vector2d point0, point1;
            
            // 获取特征点的最早观测帧和坐标
            int frame_0 = sfm_f[j].observation[0].first;
            point0 = sfm_f[j].observation[0].second;
            
            // 获取特征点的最晚观测帧和坐标
            int frame_1 = sfm_f[j].observation.back().first;
            point1 = sfm_f[j].observation.back().second;
            
            // 三角化
            Vector3d point_3d;
            triangulatePoint(Pose[frame_0], Pose[frame_1], point0, point1, point_3d);
            
            // 更新特征点状态和位置
            sfm_f[j].state = true;
            sfm_f[j].position[0] = point_3d(0);
            sfm_f[j].position[1] = point_3d(1);
            sfm_f[j].position[2] = point_3d(2);
            
            // 可选：打印调试信息
            //cout << "trangulated : " << frame_0 << " " << frame_1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
        }		
    }
		
	/*
	// 上面代码注释中提到的调试输出部分
	for (int i = 0; i < frame_num; i++)
	{
		q[i] = c_Rotation[i].transpose();  // 从相机旋转转换回世界旋转
		cout << "solvePnP  q" << " i " << i <<"  " <<q[i].w() << "  " << q[i].vec().transpose() << endl;
	}
	for (int i = 0; i < frame_num; i++)
	{
		Vector3d t_tmp;
		t_tmp = -1 * (q[i] * c_Translation[i]);  // 从相机平移转换回世界平移
		cout << "solvePnP  t" << " i " << i <<"  " << t_tmp.x() <<"  "<< t_tmp.y() <<"  "<< t_tmp.z() << endl;
	}
	*/

	// ========== 第6步：全局Bundle Adjustment优化 ==========
	ceres::Problem problem;  // 创建Ceres优化问题
	ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
	//cout << " begin full BA " << endl;

	// 1. 准备优化参数：将所有相机位姿和3D点坐标添加到优化问题中
	for (int i = 0; i < frame_num; i++)
	{
		// 将Eigen向量转换为Ceres需要的double数组格式
		c_translation[i][0] = c_Translation[i].x();
		c_translation[i][1] = c_Translation[i].y();
		c_translation[i][2] = c_Translation[i].z();
		
		c_rotation[i][0] = c_Quat[i].w();  // 四元数的实部
		c_rotation[i][1] = c_Quat[i].x();  // 四元数的虚部x
		c_rotation[i][2] = c_Quat[i].y();  // 四元数的虚部y
		c_rotation[i][3] = c_Quat[i].z();  // 四元数的虚部z
		
		// 将旋转四元数参数块添加到优化问题中
		// 使用四元数局部参数化，确保四元数在优化过程中保持单位长度
		problem.AddParameterBlock(c_rotation[i], 4, local_parameterization);
		
		// 将平移向量参数块添加到优化问题中
		problem.AddParameterBlock(c_translation[i], 3);
		
		// 设置第l帧的旋转为固定值（作为世界坐标系原点）
		if (i == l)
		{
			problem.SetParameterBlockConstant(c_rotation[i]);
		}
		
		// 设置第l帧和最后一帧的平移为固定值（保持尺度固定）
		if (i == l || i == frame_num - 1)
		{
			problem.SetParameterBlockConstant(c_translation[i]);
		}
	}

	// 2. 添加重投影误差残差块
	for (int i = 0; i < feature_num; i++)
	{
		// 只使用已成功三角化的特征点
		if (sfm_f[i].state != true)
			continue;
		
		// 遍历该特征点的所有观测
		for (int j = 0; j < int(sfm_f[i].observation.size()); j++)
		{
			int l = sfm_f[i].observation[j].first;  // 观测所在的帧索引
			
			// 创建重投影误差代价函数
			// 使用观测的x,y坐标
			ceres::CostFunction* cost_function = ReprojectionError3D::Create(
				sfm_f[i].observation[j].second.x(),
				sfm_f[i].observation[j].second.y());
			
			// 将残差块添加到优化问题中
			// 参数：代价函数，损失函数（NULL表示平方损失），参数块（旋转、平移、3D点）
			problem.AddResidualBlock(cost_function, NULL, c_rotation[l], c_translation[l], 
									sfm_f[i].position);
		}
	}

	// 3. 配置和运行求解器
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;  // 使用DENSE_SCHUR线性求解器
	//options.minimizer_progress_to_stdout = true;    // 是否输出优化过程
	options.max_solver_time_in_seconds = 0.2;        // 最大求解时间限制

	ceres::Solver::Summary summary;                  // 求解器总结信息
	ceres::Solve(options, &problem, &summary);       // 执行优化
	//std::cout << summary.BriefReport() << "\n";     // 输出简要报告

	// 4. 检查优化是否成功收敛
	if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03)
	{
		//cout << "vision only BA converge" << endl;  // 优化收敛
	}
	else
	{
		//cout << "vision only BA not converge " << endl;  // 优化未收敛
		return false;  // 返回失败
	}

	// 5. 将优化后的参数从Ceres格式转换回Eigen格式
	for (int i = 0; i < frame_num; i++)
	{
		// 更新四元数旋转
		q[i].w() = c_rotation[i][0]; 
		q[i].x() = c_rotation[i][1]; 
		q[i].y() = c_rotation[i][2]; 
		q[i].z() = c_rotation[i][3]; 
		
		q[i] = q[i].inverse();  // 从相机坐标系转换回世界坐标系
		//cout << "final  q" << " i " << i <<"  " <<q[i].w() << "  " << q[i].vec().transpose() << endl;
	}

	for (int i = 0; i < frame_num; i++)
	{
		// 更新平移向量
		T[i] = -1 * (q[i] * Vector3d(c_translation[i][0], c_translation[i][1], c_translation[i][2]));
		//cout << "final  t" << " i " << i <<"  " << T[i](0) <<"  "<< T[i](1) <<"  "<< T[i](2) << endl;
	}

	// 6. 将优化后的3D点保存到输出映射中
	for (int i = 0; i < (int)sfm_f.size(); i++)
	{
		if(sfm_f[i].state)  // 只保存成功三角化的点
			sfm_tracked_points[sfm_f[i].id] = Vector3d(sfm_f[i].position[0], 
													sfm_f[i].position[1], 
													sfm_f[i].position[2]);
	}

	return true;  // SFM成功完成

}

