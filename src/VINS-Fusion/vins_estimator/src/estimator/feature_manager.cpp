/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"

// FeaturePerId类的成员函数：返回该特征点最后一次被观测到的帧索引
int FeaturePerId::endFrame()
{
    // 起始帧 + 该特征被观测到的帧数 - 1 = 最后观测帧索引
    // 例如：start_frame=0, size()=3，则最后出现在第2帧
    return start_frame + feature_per_frame.size() - 1;
}

// FeatureManager构造函数
// 参数：
//   _Rs: 旋转矩阵数组指针，存储每帧相机到世界坐标系的旋转
FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)  // 初始化列表：将传入的旋转矩阵数组指针赋值给成员变量Rs
{
    // 初始化相机到IMU的旋转外参为单位矩阵（默认无旋转）
    // NUM_OF_CAM应该是预定义的相机数量常量（通常是2，表示双目）
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();  // 设置为单位矩阵
}

// 设置相机到IMU的外参旋转矩阵
// 参数：
//   _ric: 外参旋转矩阵数组
void FeatureManager::setRic(Matrix3d _ric[])
{
    // 遍历所有相机（双目系统通常是2个）
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];  // 复制外参矩阵
    }
}

// 清空特征管理器中的所有状态
void FeatureManager::clearState()
{
    feature.clear();  // 清空特征列表，移除所有跟踪的特征
}

// 获取可用于优化的特征数量
// 返回值：被至少4帧观测到的特征数量（这些特征适合三角化和优化）
int FeatureManager::getFeatureCount()
{
    int cnt = 0;  // 计数器
    
    // 遍历所有特征
    for (auto &it : feature)
    {
        // 更新该特征被观测的次数（即它在多少帧中被看到）
        it.used_num = it.feature_per_frame.size();
        
        // 如果该特征被至少4帧观测到，则计入有效特征
        if (it.used_num >= 4)
        {
            cnt++;
        }
    }
    return cnt;
}

// 添加新特征并检查视差，决定是否将当前帧设为关键帧
// 参数：
//   frame_count: 当前帧的索引
//   image: 当前帧检测到的特征点，map结构：feature_id -> vector<pair<相机id, 7维观测数据>>
//   td: 时间延迟
// 返回值：true表示需要将当前帧设为关键帧，false表示不需要
bool FeatureManager::addFeatureCheckParallax(int frame_count, 
                                             const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, 
                                             double td)
{
    // 调试信息：输入的特征数量和当前管理的特征数量
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    
    // 初始化统计变量
    double parallax_sum = 0;  // 所有特征视差总和
    int parallax_num = 0;     // 参与视差计算的特征数量
    last_track_num = 0;       // 在当前帧中跟踪到的旧特征数量
    last_average_parallax = 0; // 上一帧的平均视差
    new_feature_num = 0;      // 新出现的特征数量
    long_track_num = 0;       // 长轨迹特征数量（被至少4帧跟踪）
    
    // 遍历当前帧检测到的所有特征
    for (auto &id_pts : image)
    {
        // 创建当前帧的特征观测对象
        // id_pts.second[0]是左目观测（相机id=0），第二个参数是7维观测数据
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        
        // 验证：确认第一个观测确实来自左目相机（相机id=0）
        assert(id_pts.second[0].first == 0);
        
        // 如果是双目观测（有左右目两个观测数据）
        if(id_pts.second.size() == 2)
        {
            // 添加右目观测数据
            f_per_fra.rightObservation(id_pts.second[1].second);
            // 验证：确认第二个观测来自右目相机（相机id=1）
            assert(id_pts.second[1].first == 1);
        }
        
        // 获取特征ID
        int feature_id = id_pts.first;
        
        // 在现有特征列表中查找是否已有相同ID的特征
        auto it = find_if(feature.begin(), feature.end(), 
                         [feature_id](const FeaturePerId &it)
                         {
                             return it.feature_id == feature_id;
                         });
        
        // 如果特征不存在于当前管理的特征列表中（新特征）
        if (it == feature.end())
        {
            // 创建新特征跟踪对象，起始帧为当前帧
            feature.push_back(FeaturePerId(feature_id, frame_count));
            // 将当前帧观测添加到该特征的观测序列中
            feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;  // 新特征计数增加
        }
        // 如果特征已存在（旧特征，继续跟踪）
        else if (it->feature_id == feature_id)
        {
            // 将当前帧观测添加到该特征的观测序列
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;  // 跟踪到的旧特征计数增加
            
            // 如果该特征已被至少4帧跟踪，计入长轨迹特征
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;
        }
    }
    
    // 关键帧选择策略：判断当前帧是否应该设为关键帧
    // 以下条件满足任一，则返回true（将当前帧设为关键帧）
    
    // 条件1：帧数少于2帧（系统刚启动，需要关键帧）
    // 条件2：跟踪到的旧特征少于20个（跟踪不稳定，需要关键帧）
    // 条件3：长轨迹特征少于40个（特征跟踪长度不足，需要关键帧）
    // 条件4：新特征占比超过50%（场景变化大，需要关键帧）
    // 注意：这里使用了||操作符，任意条件满足即返回true
    if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
        return true;
    
    // 计算平均视差（仅考虑满足条件的特征）
    for (auto &it_per_id : feature)
    {
        // 条件：特征起始帧在当前帧的前两帧或更早，且特征的最后观测帧包含前一帧
        // 即：该特征至少被当前帧的前一帧和当前帧都观测到
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            // 计算该特征的补偿后视差并累加
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;  // 参与计算的特征数增加
        }
    }
    
    // 如果没有满足条件的特征计算视差
    if (parallax_num == 0)
    {
        // 视差无法计算，将当前帧设为关键帧
        return true;
    }
    else
    {
        // 输出调试信息：视差总和和参与计算的特征数
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        // 输出调试信息：当前平均视差（乘以焦距转换为像素单位）
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        
        // 记录平均视差（转换为像素单位）
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        
        // 关键帧判断：平均视差是否大于等于最小视差阈值
        // MIN_PARALLAX是预定义的最小视差阈值
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

// 获取两帧之间的对应特征点对（三维点坐标）
// 参数：
//   frame_count_l: 左帧索引（通常是较早的帧）
//   frame_count_r: 右帧索引（通常是较晚的帧）
// 返回值：对应特征点在两帧中的三维坐标对列表
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;  // 存储对应点对的容器
    
    // 遍历所有特征点
    for (auto &it : feature)
    {
        // 检查特征是否同时存在于两帧中
        // 条件：特征起始帧 <= 左帧索引，且特征结束帧 >= 右帧索引
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();  // 初始化两个三维点
            
            // 计算特征在特征观测列表中的索引
            // 因为特征观测列表是按帧顺序存储的
            int idx_l = frame_count_l - it.start_frame;  // 左帧在观测列表中的索引
            int idx_r = frame_count_r - it.start_frame;  // 右帧在观测列表中的索引

            // 获取左帧中的三维点坐标（在相机坐标系下）
            a = it.feature_per_frame[idx_l].point;

            // 获取右帧中的三维点坐标（在相机坐标系下）
            b = it.feature_per_frame[idx_r].point;
            
            // 将点对添加到结果列表
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

// 设置特征点的深度值（从优化变量中）
// 参数：
//   x: 优化变量向量，包含特征深度的逆（逆深度参数化）
void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;  // 特征索引，从-1开始，因为后面使用++feature_index
    
    // 遍历所有特征点
    for (auto &it_per_id : feature)
    {
        // 更新该特征被观测的次数
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        
        // 只处理被至少4帧观测到的特征（满足三角化条件）
        if (it_per_id.used_num < 4)
            continue;

        // 从优化变量x中获取深度的逆，并转换为深度值
        // 注意：这里使用逆深度参数化，所以需要取倒数
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        
        // ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        
        // 检查深度值的有效性
        if (it_per_id.estimated_depth < 0)
        {
            // 深度为负，标记为三角化失败
            it_per_id.solve_flag = 2;
        }
        else
            // 深度有效，标记为三角化成功
            it_per_id.solve_flag = 1;
    }
}

// 移除三角化失败的特征点
void FeatureManager::removeFailures()
{
    // 使用两个迭代器遍历特征列表，方便删除操作
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;  // 提前保存下一个迭代器，因为删除当前迭代器会使it失效
        
        // 如果特征三角化失败（solve_flag == 2），从列表中删除
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

// 清除所有特征点的深度估计值
void FeatureManager::clearDepth()
{
    // 遍历所有特征点，将深度估计值设为-1（表示未估计）
    for (auto &it_per_id : feature)
        it_per_id.estimated_depth = -1;
}

// 获取所有特征点的深度向量（用于优化）
// 返回值：深度向量，使用逆深度参数化
VectorXd FeatureManager::getDepthVector()
{
    // 创建深度向量，大小为有效特征数量
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;  // 特征索引
    
    // 遍历所有特征点
    for (auto &it_per_id : feature)
    {
        // 更新该特征被观测的次数
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        
        // 只处理被至少4帧观测到的特征
        if (it_per_id.used_num < 4)
            continue;
        
// 条件编译：通常使用逆深度参数化
#if 1
        // 将深度转换为逆深度存储（优化通常使用逆深度）
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        // 或者直接存储深度值（较少使用）
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

// 使用线性三角化方法恢复特征点的三维坐标
// 参数：
//   Pose0, Pose1: 两帧相机的投影矩阵 [R|t]（3x4）
//   point0, point1: 两帧中的特征点坐标（归一化平面坐标或像素坐标）
//   point_3d: 输出的三维点坐标（相机坐标系或世界坐标系）
void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    // 构建线性方程组的设计矩阵（4x4）
    // 原理：从两帧的投影方程推导出的线性系统
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    
    // 第一行：从第一帧投影方程推导
    // 公式：u0 * (P0第三行) - (P0第一行) = 0
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    
    // 第二行：从第一帧投影方程推导
    // 公式：v0 * (P0第三行) - (P0第二行) = 0
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    
    // 第三行：从第二帧投影方程推导
    // 公式：u1 * (P1第三行) - (P1第一行) = 0
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    
    // 第四行：从第二帧投影方程推导
    // 公式：v1 * (P1第三行) - (P1第二行) = 0
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    
    // 三角化后的点（齐次坐标，4维）
    Eigen::Vector4d triangulated_point;
    
    // 使用SVD分解求解线性系统
    // 设计矩阵是一个4x4的矩阵，我们求解Ax=0的最小二乘解
    // SVD分解：A = UΣV^T，解是V的最后一列（对应最小奇异值）
    triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    
    // 将齐次坐标转换为三维坐标（除以最后一维）
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

// 使用PnP（Perspective-n-Point）方法求解相机位姿
// 参数：
//   R: 输入输出参数，作为初始猜测传入，输出求解后的旋转矩阵（世界坐标系到IMU坐标系）
//   P: 输入输出参数，作为初始猜测传入，输出求解后的平移向量（世界坐标系到IMU坐标系）
//   pts2D: 二维图像点（归一化平面坐标）
//   pts3D: 对应的三维空间点（世界坐标系）
// 返回值：求解是否成功
bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, 
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // 将世界坐标系到IMU坐标系的变换转换为世界坐标系到相机坐标系的变换
    // 原始：R和P是世界坐标系到IMU坐标系的变换（w_T_imu）
    // 需要转换为世界坐标系到相机坐标系的变换（w_T_cam）
    // 公式：cam_T_w = (w_T_cam)^-1 = (w_T_imu * imu_T_cam)^-1 = cam_T_imu * imu_T_w
    // 由于我们只有w_T_imu，假设imu_T_cam是单位变换（相机和IMU坐标系重合），则：
    // cam_T_w = (w_T_imu)^-1 = imu_T_w
    R_initial = R.inverse();          // 旋转矩阵的逆等于转置
    P_initial = -(R_initial * P);     // 平移向量的变换

    // 检查是否有足够的点进行PnP求解（至少4个点）
    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    
    // OpenCV相关变量
    cv::Mat r, rvec, t, D, tmp_r;
    // 将Eigen旋转矩阵转换为OpenCV矩阵
    cv::eigen2cv(R_initial, tmp_r);
    // 将旋转矩阵转换为旋转向量（罗德里格斯表示），因为OpenCV的solvePnP需要旋转向量
    cv::Rodrigues(tmp_r, rvec);
    // 将Eigen平移向量转换为OpenCV矩阵
    cv::eigen2cv(P_initial, t);
    // 创建单位内参矩阵（因为使用归一化平面坐标，不需要内参）
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
    
    bool pnp_succ;
    // 使用OpenCV的solvePnP求解相机位姿
    // 参数：
    //   pts3D: 3D点（世界坐标系）
    //   pts2D: 2D点（归一化平面坐标）
    //   K: 内参矩阵（单位矩阵）
    //   D: 畸变系数（空）
    //   rvec: 输出的旋转向量（世界坐标系到相机坐标系）
    //   t: 输出的平移向量（世界坐标系到相机坐标系）
    //   1: 使用迭代法求解（CV_ITERATIVE）
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    // 也可以使用RANSAC版本（更鲁棒）：
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    // 检查PnP是否成功
    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    
    // 将旋转向量转换回旋转矩阵
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    
    // 将OpenCV矩阵转换回Eigen矩阵
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // 将世界坐标系到相机坐标系的变换转换回世界坐标系到IMU坐标系的变换
    // 公式：w_T_imu = (cam_T_w)^-1
    R = R_pnp.transpose();        // 旋转矩阵的逆等于转置
    P = R * (-T_pnp);             // 平移向量的变换

    return true;
}

// 通过PnP初始化新帧的位姿
// 参数：
//   frameCnt: 当前帧索引
//   Ps[]: 位置数组（世界坐标系到IMU坐标系）
//   Rs[]: 旋转矩阵数组（世界坐标系到IMU坐标系）
//   tic[]: 相机到IMU的平移外参
//   ric[]: 相机到IMU的旋转外参
void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    // 只在frameCnt > 0时执行（至少有一帧历史帧）
    if(frameCnt > 0)
    {
        vector<cv::Point2f> pts2D;  // 当前帧的2D观测点
        vector<cv::Point3f> pts3D;  // 对应的3D空间点（世界坐标系）
        
        // 遍历所有特征点
        for (auto &it_per_id : feature)
        {
            // 只使用已经成功三角化（深度>0）的特征点
            if (it_per_id.estimated_depth > 0)
            {
                // 计算特征点在当前帧观测列表中的索引
                int index = frameCnt - it_per_id.start_frame;
                
                // 确保特征点确实在当前帧有观测
                if((int)it_per_id.feature_per_frame.size() >= index + 1)
                {
                    // 重建3D点（世界坐标系）：
                    // 1. 使用第一帧观测和估计的深度重建点在相机坐标系中的位置
                    // 2. 将点从第一帧相机坐标系转换到第一帧IMU坐标系（通过外参）
                    // 3. 将点从第一帧IMU坐标系转换到世界坐标系（通过第一帧位姿）
                    Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];

                    // 将3D点（世界坐标系）和2D点（当前帧相机坐标系）添加到PnP输入
                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), 
                                        it_per_id.feature_per_frame[index].point.y());
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d); 
                }
            }
        }
        
        // 计算上一帧的相机位姿（世界坐标系到相机坐标系），作为PnP的初始猜测
        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // 公式：w_T_cam = w_T_imu * imu_T_cam
        // imu_T_cam = [ric^T, -ric^T*tic] 的逆
        // 但这里计算的是：w_T_cam = (w_T_imu) * (imu_T_cam)^-1
        RCam = Rs[frameCnt - 1] * ric[0];                    // 旋转部分
        PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1]; // 平移部分

        // 使用PnP求解当前帧的相机位姿
        if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))
        {
            // 将求解得到的相机位姿（世界坐标系到相机坐标系）转换回IMU位姿（世界坐标系到IMU坐标系）
            // 公式：w_T_imu = w_T_cam * cam_T_imu = w_T_cam * (imu_T_cam)^-1
            Rs[frameCnt] = RCam * ric[0].transpose();  // 旋转部分
            Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;  // 平移部分

            // 输出调试信息（已注释）
            Eigen::Quaterniond Q(Rs[frameCnt]);
            //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
        }
    }
}

// 对特征点进行三角化（恢复三维深度）
// 参数：
//   frameCnt: 当前帧索引
//   Ps[]: 位置数组（世界坐标系到IMU坐标系）
//   Rs[]: 旋转矩阵数组（世界坐标系到IMU坐标系）
//   tic[]: 相机到IMU的平移外参
//   ric[]: 相机到IMU的旋转外参
void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    // 遍历所有特征点
    for (auto &it_per_id : feature)
    {
        // 如果该特征已经估计过深度（大于0），跳过
        if (it_per_id.estimated_depth > 0)
            continue;

        // 情况1：双目三角化（如果系统是双目且特征的第一帧是双目观测）
        // STEREO应该是预定义的宏，表示是否为双目系统
        if(STEREO && it_per_id.feature_per_frame[0].is_stereo)
        {
            int imu_i = it_per_id.start_frame;  // 特征起始帧索引
            
            // 构建左相机投影矩阵（世界坐标系到左相机坐标系）
            Eigen::Matrix<double, 3, 4> leftPose;
            // 计算左相机在世界坐标系中的位置：Ps[imu_i] + Rs[imu_i] * tic[0]
            // tic[0]是IMU到左相机的平移，Rs[imu_i]将其转到世界坐标系
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            // 计算左相机在世界坐标系中的旋转：Rs[imu_i] * ric[0]
            // ric[0]是IMU到左相机的旋转
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            // 投影矩阵公式：P = K[R|t]，这里使用[R^T|-R^T*t]的形式（因为后续需要变换）
            leftPose.leftCols<3>() = R0.transpose();          // 旋转部分：R^T
            leftPose.rightCols<1>() = -R0.transpose() * t0;   // 平移部分：-R^T * t
            
            // 构建右相机投影矩阵（世界坐标系到右相机坐标系）
            Eigen::Matrix<double, 3, 4> rightPose;
            // 右相机在世界坐标系中的位置
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
            // 右相机在世界坐标系中的旋转
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            // 获取左右目的特征点坐标（归一化平面坐标）
            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;  // 三角化后的三维点（左相机坐标系）
            // 取特征点的前两维（x,y），忽略z（因为归一化平面坐标z=1）
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[0].pointRight.head(2);

            // 使用线性三角化方法恢复三维点
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            
            // 将三角化的点转换到左相机坐标系
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            
            // 深度就是z坐标（在左相机坐标系下）
            double depth = localPoint.z();
            
            // 如果深度为正，保存估计值；否则设为初始深度
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;  // INIT_DEPTH是预定义的初始深度值
            
            continue;  // 双目三角化完成，继续下一个特征
        }
        // 情况2：单目两帧三角化（特征被至少两帧观测到）
        else if(it_per_id.feature_per_frame.size() > 1)
        {
            int imu_i = it_per_id.start_frame;  // 特征起始帧索引
            
            // 构建第一帧相机的投影矩阵
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;

            // 构建第二帧相机的投影矩阵
            imu_i++;  // 移动到下一帧
            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            // 获取两帧中的特征点坐标
            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[1].point.head(2);
            
            // 三角化
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            
            // 计算深度
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            
            // 保存深度估计
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            
            continue;  // 两帧三角化完成，继续下一个特征
        }
        
        // 情况3：多帧三角化（特征被多帧观测到，使用SVD方法）
        // 更新特征被观测的次数
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        
        // 如果观测次数小于4，跳过（需要足够多的观测才能进行稳定的三角化）
        if (it_per_id.used_num < 4)
            continue;

        // 准备SVD三角化
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;  // imu_j将从imu_i开始递增

        // 构建线性方程系统：A * X = 0，其中X是特征点的齐次坐标
        // 每帧观测贡献两个方程，所以总方程数为2 * 观测次数
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;  // 当前行索引

        // 第一帧的投影矩阵（参考帧）
        Eigen::Matrix<double, 3, 4> P0;
        // 第一帧相机在世界坐标系中的位置和旋转
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        // 将第一帧设为参考系，所以其投影矩阵设为[I|0]
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        // 遍历该特征的所有观测帧
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;  // 当前帧索引

            // 计算当前帧相对于第一帧的变换
            // 当前帧相机在世界坐标系中的位置和旋转
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            
            // 计算相对变换：从第一帧到当前帧的变换
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);  // 平移
            Eigen::Matrix3d R = R0.transpose() * R1;         // 旋转
            
            // 构建当前帧在第一帧坐标系下的投影矩阵
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();          // 旋转部分
            P.rightCols<1>() = -R.transpose() * t;    // 平移部分
            
            // 获取归一化特征坐标（三维向量，但z=1）
            Eigen::Vector3d f = it_per_frame.point.normalized();
            
            // 构建线性方程：
            // 对于投影方程：u = P * X，其中X是齐次坐标
            // 可以推导出两个线性方程：
            // u * (P的第三行) = (P的第一行) * X
            // v * (P的第三行) = (P的第二行) * X
            // 整理得到：
            // f[0] * P.row(2) - f[2] * P.row(0) = 0
            // f[1] * P.row(2) - f[2] * P.row(1) = 0
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            // 如果是第一帧本身，跳过（因为P是单位矩阵，不会增加有效约束）
            if (imu_i == imu_j)
                continue;
        }
        
        // 确保行数正确
        ROS_ASSERT(svd_idx == svd_A.rows());
        
        // 使用SVD求解齐次线性系统 A * X = 0 的最小二乘解
        // A的奇异值分解：A = U * Σ * V^T
        // 解是V的最后一列（对应最小奇异值）
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        
        // 从齐次坐标中提取深度（参考第一帧相机坐标系）
        // 注意：这里假设特征点在第一帧相机坐标系中的齐次坐标为[x,y,z,1]^T
        // 通过svd_V[2]/svd_V[3]得到深度z
        double svd_method = svd_V[2] / svd_V[3];
        
        // 保存深度估计
        it_per_id.estimated_depth = svd_method;

        // 如果深度估计值太小（小于0.1），设为初始深度
        // 这是为了处理三角化失败或深度估计不可靠的情况
        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }
    }
}

// 移除被标记为异常值的特征点
// 参数：
//   outlierIndex: 包含异常值特征ID的集合
void FeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;  // 用于在outlierIndex中查找的迭代器
    // 遍历特征列表，使用两个迭代器以便安全删除
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;  // 提前保存下一个迭代器，因为删除当前迭代器会使it失效
        
        int index = it->feature_id;  // 获取当前特征的ID
        // 在异常值集合中查找当前特征ID
        itSet = outlierIndex.find(index);
        
        // 如果找到了（说明该特征被标记为异常值）
        if(itSet != outlierIndex.end())
        {
            feature.erase(it);  // 从特征列表中删除该特征
            //printf("remove outlier %d \n", index);
        }
    }
}

// 移除滑动窗口中最老的帧，并调整剩余特征的深度（用于边缘化操作）
// 参数：
//   marg_R, marg_P: 被边缘化帧的旋转和平移（世界坐标系到该帧IMU坐标系）
//   new_R, new_P: 新参考帧的旋转和平移（世界坐标系到新参考帧IMU坐标系）
void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, 
                                          Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    // 遍历所有特征，使用两个迭代器以便安全删除
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;  // 提前保存下一个迭代器

        // 如果特征不是从第0帧开始观测的（即不是从被移除的帧开始的）
        if (it->start_frame != 0)
            it->start_frame--;  // 特征起始帧索引减1（因为第0帧被移除了）
        else  // 特征是从第0帧开始观测的（即被移除帧中首次观测到）
        {
            // 获取该特征在第0帧中的归一化坐标
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            // 删除第0帧的观测（因为第0帧将被移除）
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            
            // 如果删除后特征观测数少于2帧，则直接删除该特征（无法继续跟踪）
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else  // 如果还有至少2帧观测，需要调整深度估计
            {
                // 将该特征在第0帧的三维点还原（使用估计的深度）
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                // 将点从第0帧相机坐标系转换到世界坐标系
                // 公式：P_w = R0 * P_c0 + t0
                // 这里marg_R和marg_P就是第0帧的R和t（世界坐标系到第0帧IMU坐标系）
                // 但注意：pts_i是在第0帧相机坐标系下的，而marg_R和marg_P是IMU坐标系的
                // 这里假设相机和IMU之间没有外参或外参已考虑在内（简化处理）
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                
                // 将世界坐标系下的点转换到新参考帧（第1帧）的相机坐标系
                // 公式：P_c1 = R1^T * (P_w - t1)
                // new_R和new_P是新参考帧（第1帧）的IMU位姿
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                
                // 在新参考帧中的深度就是z坐标
                double dep_j = pts_j(2);
                
                // 如果深度为正，更新深度估计；否则设为初始深度
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // 边缘化后移除跟踪丢失的特征（注释掉的代码）
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
}

// 移除滑动窗口中最老的帧（简化的版本，不进行深度调整）
void FeatureManager::removeBack()
{
    // 遍历所有特征
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;  // 提前保存下一个迭代器

        // 如果特征不是从第0帧开始观测的
        if (it->start_frame != 0)
            it->start_frame--;  // 起始帧索引减1
        else  // 特征是从第0帧开始观测的
        {
            // 删除第0帧的观测
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            // 如果删除后没有观测了，则删除该特征
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

// 移除滑动窗口中最新的帧（特定情况使用）
// 参数：
//   frame_count: 要移除的帧索引
void FeatureManager::removeFront(int frame_count)
{
    // 遍历所有特征
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;  // 提前保存下一个迭代器

        // 如果特征起始帧就是要移除的帧
        if (it->start_frame == frame_count)
        {
            it->start_frame--;  // 起始帧索引减1
        }
        else
        {
            // 计算要删除的观测在特征观测列表中的索引
            // WINDOW_SIZE是滑动窗口大小
            int j = WINDOW_SIZE - 1 - it->start_frame;
            
            // 如果特征的最后一帧在要移除的帧之前，跳过
            if (it->endFrame() < frame_count - 1)
                continue;
            
            // 删除指定帧的观测
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            
            // 如果删除后没有观测了，则删除该特征
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

// 计算补偿后的视差（用于关键帧选择）
// 参数：
//   it_per_id: 特征ID的引用
//   frame_count: 当前帧索引
// 返回值：补偿后的视差值
double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    // 检查倒数第二帧是否是关键帧
    // 计算倒数第二帧和倒数第三帧之间的视差
    
    // 获取倒数第二帧的特征观测（frame_count-2）
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    // 获取倒数第一帧的特征观测（frame_count-1）
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;  // 存储最终视差值
    Vector3d p_j = frame_j.point;  // 倒数第一帧的特征点（归一化坐标）

    double u_j = p_j(0);  // 倒数第一帧的x坐标
    double v_j = p_j(1);  // 倒数第一帧的y坐标

    Vector3d p_i = frame_i.point;  // 倒数第二帧的特征点（归一化坐标）
    Vector3d p_i_comp;  // 补偿后的倒数第二帧特征点

    // 注释掉的代码：理论上应该考虑相机和IMU之间的外参以及帧间旋转
    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    
    // 简化处理：假设没有旋转补偿（实际系统中可能需要考虑）
    p_i_comp = p_i;
    
    // 计算原始视差
    double dep_i = p_i(2);  // 原始深度（归一化坐标中通常为1）
    double u_i = p_i(0) / dep_i;  // 原始x坐标（除以深度，确保在归一化平面）
    double v_i = p_i(1) / dep_i;  // 原始y坐标
    double du = u_i - u_j, dv = v_i - v_j;  // 原始坐标差

    // 计算补偿后的视差
    double dep_i_comp = p_i_comp(2);  // 补偿后的深度
    double u_i_comp = p_i_comp(0) / dep_i_comp;  // 补偿后的x坐标
    double v_i_comp = p_i_comp(1) / dep_i_comp;  // 补偿后的y坐标
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;  // 补偿后的坐标差

    // 选择原始视差和补偿后视差中较小的一个，取最大值（与0比较）
    // 这是因为视差可能为负（取决于运动方向），我们关心的是视差的大小
    // 使用较小的视差是保守策略，确保关键帧选择不会过于频繁
    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}