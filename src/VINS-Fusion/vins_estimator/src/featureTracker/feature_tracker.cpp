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

#include "feature_tracker.h"

// 检查点是否在图像边界内的成员函数
// 参数：pt - 要检查的点坐标（像素坐标）
// 返回值：true表示点在边界内，false表示点在边界外
bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;  // 边界大小，通常为1像素，避免特征点在图像边缘不稳定
    
    // 将浮点坐标四舍五入为整型坐标
    int img_x = cvRound(pt.x);  // cvRound: OpenCV的四舍五入函数
    int img_y = cvRound(pt.y);
    
    // 检查点是否在图像有效区域内（减去边界大小）
    // col: 图像列数（宽度），row: 图像行数（高度）
    return BORDER_SIZE <= img_x &&           // 点在左边界内
           img_x < col - BORDER_SIZE &&      // 点在右边界内
           BORDER_SIZE <= img_y &&           // 点在上边界内
           img_y < row - BORDER_SIZE;        // 点在下边界内
}

// 计算两点之间欧氏距离的全局函数
// 参数：pt1, pt2 - 两个点的坐标
// 返回值：两点之间的欧氏距离
double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    // 调试输出，被注释掉了，可用于调试
    // printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    
    // 计算x和y方向的差值
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    
    // 计算欧氏距离：√(dx² + dy²)
    return sqrt(dx * dx + dy * dy);
}

// 根据状态向量缩减点向量
// 参数：v - 要缩减的点向量（输入输出参数）
//       status - 状态向量，每个元素表示对应点的状态（1保留，0剔除）
// 功能：只保留status为1的点，移除status为0的点
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;  // 新向量中的当前位置索引
    
    // 遍历原始向量
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])  // 如果状态为1（true），保留该点
            v[j++] = v[i];  // 将点复制到新位置，然后j递增
    
    v.resize(j);  // 调整向量大小为j，删除多余元素
}

// 根据状态向量缩减ID向量（重载版本）
// 参数：v - 要缩减的ID向量（输入输出参数）
//       status - 状态向量，每个元素表示对应ID的状态
void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])       // 状态为1，保留该ID
            v[j++] = v[i];   // 复制ID到新位置
    v.resize(j);             // 调整大小
}

// 特征跟踪器的构造函数
FeatureTracker::FeatureTracker()
{
    // 初始化双目标志为0（单目模式）
    stereo_cam = 0;      // 0: 单目，1: 双目
    
    // 初始化特征点ID为0
    n_id = 0;            // 下一个可用的特征点ID，从0开始
    
    // 初始化预测标志为false（当前没有预测信息）
    hasPrediction = false;  // 用于指示是否有来自IMU或其他传感器的预测
}

// 设置特征提取掩码
// 功能：创建一个掩码，避免在已有特征点附近提取新特征点
void FeatureTracker::setMask()
{
    // 创建与图像大小相同的掩码，初始化为全白（255）
    // CV_8UC1: 8位无符号单通道图像
    // cv::Scalar(255): 白色，表示可以提取特征的区域
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));
    
    // 创建临时向量，存储每个特征点的跟踪计数、位置和ID
    // pair结构：<跟踪计数, <点坐标, 特征点ID>>
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    
    // 遍历当前帧的所有特征点
    for (unsigned int i = 0; i < cur_pts.size(); i++)
        // 将数据添加到临时向量中
        cnt_pts_id.push_back(make_pair(
            track_cnt[i],                // 跟踪计数（被连续跟踪的帧数）
            make_pair(cur_pts[i], ids[i]) // 点坐标和ID
        ));
    
    // 对临时向量进行排序，按跟踪计数降序排列
    // 使用lambda表达式定义比较函数：按第一元素（跟踪计数）降序
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), 
         [](const pair<int, pair<cv::Point2f, int>> &a, 
            const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;  // 降序排列
         });
    
    // 清空原始向量，准备重新填充
    cur_pts.clear();
    ids.clear();
    track_cnt.clear();
    
    // 遍历排序后的特征点（优先保留跟踪时间长的点）
    for (auto &it : cnt_pts_id)
    {
        // 检查当前点在掩码中是否标记为可提取（255表示可提取）
        if (mask.at<uchar>(it.second.first) == 255)
        {
            // 保留该特征点
            cur_pts.push_back(it.second.first);  // 添加点坐标
            ids.push_back(it.second.second);      // 添加ID
            track_cnt.push_back(it.first);        // 添加跟踪计数
            
            // 在掩码上以该点为中心画一个黑色圆（0表示不可提取）
            // 目的是避免在该点周围MIN_DIST范围内提取新特征点
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
            // MIN_DIST: 最小距离常数，保证特征点之间的最小像素距离
            // 0: 圆的颜色（黑色）
            // -1: 填充圆（实心圆）
        }
    }
    // 循环结束后，特征点按照跟踪时间长短重新排序
    // 跟踪时间长的特征点被优先保留，并在掩码上标记其周围区域不可提取新特征
}

// 类成员版本的欧氏距离计算函数
// 参数：pt1, pt2 - 两个点的引用
// 返回值：两点之间的欧氏距离
double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    // 调试输出，被注释掉了
    // printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    
    // 计算x和y方向的差值
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    
    // 返回欧氏距离
    return sqrt(dx * dx + dy * dy);
}

// 特征跟踪的主函数，处理输入的图像并返回跟踪结果
// 参数：
//   _cur_time: 当前帧时间戳
//   _img: 当前帧图像（左目）
//   _img1: 可选的右目图像（默认为空）
// 返回值：特征帧数据结构，包含所有跟踪到的特征点信息
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r;  // 计时器开始，用于测量整个函数执行时间
    cur_time = _cur_time;  // 记录当前帧时间戳
    cur_img = _img;        // 存储当前帧图像
    row = cur_img.rows;    // 记录图像行数（高度）
    col = cur_img.cols;    // 记录图像列数（宽度）
    cv::Mat rightImg = _img1;  // 存储右目图像（如果存在）
    
    /*
    // 直方图均衡化（CLAHE），用于增强图像对比度，被注释掉了
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    
    cur_pts.clear();  // 清空当前帧特征点容器，准备填充新的跟踪结果

    // 如果上一帧有特征点，则进行光流跟踪
    if (prev_pts.size() > 0)
    {
        TicToc t_o;  // 计时器，用于测量光流跟踪时间
        vector<uchar> status;  // 光流跟踪状态向量（1成功，0失败）
        vector<float> err;     // 光流跟踪误差
        
        // 如果有预测信息，使用预测点作为光流初始值
        if(hasPrediction)
        {
            cur_pts = predict_pts;  // 使用预测的点作为当前帧特征点的初始估计
            
            // 执行LK金字塔光流跟踪
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, 
                cv::Size(21, 21),  // 窗口大小：21x21
                1,                 // 金字塔层数：1（因为有初始预测，层数可以少）
                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), // 迭代终止条件
                cv::OPTFLOW_USE_INITIAL_FLOW);  // 使用初始流（预测点）
            
            // 统计成功跟踪的数量
            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
                if (status[i])
                    succ_num++;
            
            // 如果成功跟踪的点太少（少于10个），用更大的金字塔层数重新跟踪
            if (succ_num < 10)
               cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        }
        else
            // 没有预测信息，使用3层金字塔进行光流跟踪
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        
        // 反向光流检查：从当前帧跟踪回上一帧，确保跟踪的一致性
        if(FLOW_BACK)  // FLOW_BACK是一个编译选项或配置参数
        {
            vector<uchar> reverse_status;  // 反向跟踪状态
            vector<cv::Point2f> reverse_pts = prev_pts;  // 反向跟踪得到的点（应该是上一帧的点）
            
            // 执行反向光流跟踪
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, 
                cv::Size(21, 21), 1, 
                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), 
                cv::OPTFLOW_USE_INITIAL_FLOW);
            
            // 检查正向和反向跟踪的一致性
            for(size_t i = 0; i < status.size(); i++)
            {
                // 如果正向和反向都成功，且反向跟踪得到的点与原始点距离很近（<=0.5像素）
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;  // 保留该点
                }
                else
                    status[i] = 0;  // 剔除该点（跟踪不一致）
            }
        }
        
        // 剔除位于图像边界外的点
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        
        // 根据状态向量压缩各个容器，只保留成功跟踪的点
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());  // 输出光流跟踪耗时
        // printf("track cnt %d\n", (int)ids.size());  // 调试输出跟踪到的点数量
    }

    // 增加所有点的跟踪计数（包括新提取的点）
    for (auto &n : track_cnt)
        n++;

    // 提取新特征点（总是执行，因为if(1)恒为真）
    if (1)
    {
        // rejectWithF();  // 使用基础矩阵剔除外点（被注释掉了）
        
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();  // 设置掩码，避免在已有特征点附近提取新特征
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        
        // 计算还需要提取多少新特征点
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {
            // 检查掩码是否有效
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            
            // 使用goodFeaturesToTrack提取新的特征点（Shi-Tomasi角点）
            cv::goodFeaturesToTrack(cur_img, n_pts, 
                MAX_CNT - cur_pts.size(),  // 最大特征点数 - 当前已有点数
                0.01,      // 质量水平阈值（特征点最小质量）
                MIN_DIST,  // 特征点之间的最小距离
                mask);     // 掩码，避免在已有特征点附近提取
        }
        else
            n_pts.clear();  // 不需要提取新特征点
        
        ROS_DEBUG("detect feature costs: %f ms", t_t.toc());

        // 将新提取的特征点添加到当前帧特征点集合中
        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);    // 添加点坐标
            ids.push_back(n_id++);   // 分配新的ID，然后ID自增
            track_cnt.push_back(1);  // 新点的跟踪计数初始化为1
        }
        // printf("feature cnt after add %d\n", (int)ids.size());  // 调试输出
    }

    // 对当前帧左图特征点进行去畸变，得到归一化平面坐标
    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
    
    // 计算特征点速度（归一化平面上的速度）
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    // 如果是双目相机且有右目图像，进行右目特征点跟踪
    if(!_img1.empty() && stereo_cam)
    {
        // 清空右目相关容器
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        
        // 如果左图有特征点，则在右图中跟踪对应的点
        if(!cur_pts.empty())
        {
            // printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;  // 用于反向检查
            vector<uchar> status, statusRightLeft;  // 跟踪状态
            vector<float> err;  // 跟踪误差
            
            // 从左图到右图的LK光流跟踪
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
            
            // 反向检查：从右图跟踪回左图
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                
                // 一致性检查
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;  // 保留
                    else
                        status[i] = 0;  // 剔除
                }
            }

            // 右目的ID与左目相同（对应同一个3D点）
            ids_right = ids;
            
            // 根据状态压缩右目相关向量
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            
            /*
            // 如果右目跟踪失败，也可以从左边剔除该点（被注释掉了）
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            
            // 对右目特征点去畸变
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            
            // 计算右目特征点速度
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        
        // 更新右目上一帧信息
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    
    // 如果需要显示跟踪结果，绘制跟踪图像
    if(SHOW_TRACK)
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    // 更新帧间信息（为下一帧做准备）
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;  // 重置预测标志

    // 更新左图特征点映射（用于下一帧绘制跟踪线）
    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    // 构建返回的特征帧数据结构
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    
    // 处理左目特征点
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];  // 特征点ID
        
        // 归一化平面坐标 (x, y, 1)
        double x, y, z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        
        // 像素坐标 (u, v)
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        
        int camera_id = 0;  // 左目相机ID
        
        // 归一化平面速度
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        // 构建7维向量：[x, y, z, u, v, vx, vy]
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        
        // 添加到特征帧中：feature_id -> [(camera_id, 特征信息)]
        featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    }

    // 处理右目特征点（如果是双目）
    if (!_img1.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            
            // 右目归一化平面坐标
            double x, y, z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            
            // 右目像素坐标
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            
            int camera_id = 1;  // 右目相机ID
            
            // 右目归一化平面速度
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            // 构建7维特征向量
            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            
            // 添加到特征帧中（同一个feature_id可能对应左右目两个观测）
            featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
        }
    }

    // printf("feature track whole time %f\n", t_r.toc());  // 输出整个跟踪过程耗时
    return featureFrame;  // 返回特征帧数据
}
// 使用基础矩阵(Fundamental Matrix)进行外点剔除
// 通过RANSAC算法计算基础矩阵，剔除不符合极线约束的匹配点
void FeatureTracker::rejectWithF()
{
    // 基础矩阵至少需要8对匹配点（8点法），所以检查当前点数是否足够
    if (cur_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");  // 输出调试信息，开始RANSAC计算基础矩阵
        TicToc t_f;  // 计时器，用于测量RANSAC计算耗时
        
        // 创建两个向量存储去畸变并投影到虚拟针孔相机平面的点
        vector<cv::Point2f> un_cur_pts(cur_pts.size());   // 当前帧处理后的点
        vector<cv::Point2f> un_prev_pts(prev_pts.size()); // 上一帧处理后的点
        
        // 遍历所有特征点对
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;  // 临时变量，存储投影后的3D点
            
            // 对当前帧特征点进行去畸变并投影到归一化平面
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            // 将归一化坐标转换为像素坐标（使用焦距FOCAL_LENGTH和图像中心）
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());  // 存储处理后的当前帧点

            // 对上一帧对应特征点进行同样的处理
            m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());  // 存储处理后的上一帧点
        }

        vector<uchar> status;  // 状态向量，标记哪些点是内点（1）和外点（0）
        
        // 使用RANSAC算法计算基础矩阵，并得到内点/外点状态
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, 
                              cv::FM_RANSAC,   // 使用RANSAC方法
                              F_THRESHOLD,     // 阈值，用于判断点是否满足极线约束（像素单位）
                              0.99,            // 置信度，RANSAC算法参数
                              status);         // 输出状态向量
        
        int size_a = cur_pts.size();  // 记录剔除前的特征点数量
        
        // 根据状态向量压缩各个容器，只保留内点（status=1的点）
        reduceVector(prev_pts, status);    // 压缩上一帧点
        reduceVector(cur_pts, status);     // 压缩当前帧点
        reduceVector(cur_un_pts, status);  // 压缩当前帧去畸变点
        reduceVector(ids, status);         // 压缩特征点ID
        reduceVector(track_cnt, status);   // 压缩跟踪计数
        
        // 输出剔除结果信息：原始数量 -> 处理后数量 : 内点比例
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());  // 输出RANSAC计算耗时
    }
}

// 从标定文件读取相机内参和畸变参数
// 参数：calib_file - 标定文件路径向量，可以包含一个（单目）或两个（双目）文件
void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    // 遍历所有标定文件（通常为1或2个，分别对应左目和右目）
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());  // 输出信息
        
        // 使用CameraFactory从YAML文件生成相机模型
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);  // 将相机模型指针添加到m_camera向量中
    }
    
    // 如果有两个标定文件，则设置stereo_cam标志为1（双目系统）
    if (calib_file.size() == 2)
        stereo_cam = 1;
}

// 显示去畸变效果（调试函数）
// 参数：name - 显示窗口的名称
void FeatureTracker::showUndistortion(const string &name)
{
    // 创建一个大一些的图像用于显示去畸变效果（原始图像大小加上600像素的边框）
    cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
    
    vector<Eigen::Vector2d> distortedp;   // 存储原始像素坐标
    vector<Eigen::Vector2d> undistortedp; // 存储去畸变后的归一化坐标
    
    // 遍历原始图像的每一个像素
    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            Eigen::Vector2d a(i, j);  // 原始像素坐标
            Eigen::Vector3d b;        // 去畸变后的归一化坐标
            
            // 调用相机模型的liftProjective函数进行去畸变
            m_camera[0]->liftProjective(a, b);
            
            // 存储原始坐标和去畸变后的归一化坐标
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            // printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z()); // 调试输出
        }
    
    // 将去畸变后的坐标映射到显示图像上
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);  // 创建齐次坐标向量
        
        // 将归一化坐标转换为像素坐标（假设焦距为FOCAL_LENGTH）
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
        pp.at<float>(2, 0) = 1.0;  // 齐次坐标的z分量为1
        
        // 将原始图像的像素值复制到去畸变图像的对应位置（加上300的偏移，让图像居中显示）
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && 
            pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = 
                cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            // ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
            // 如果映射超出边界，可以选择输出错误信息（这里被注释了）
        }
    }
    
    // 显示去畸变图像（需要时可以取消注释）
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(0);
}

// 对特征点进行去畸变，返回归一化平面坐标
// 参数：pts - 输入的像素坐标特征点
//       cam - 相机模型指针
// 返回值：去畸变后的归一化平面坐标（z=1平面）
vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;  // 存储结果的向量
    
    // 遍历所有输入点
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);  // 像素坐标转换为Eigen向量
        Eigen::Vector3d b;  // 去畸变后的3D点（在相机坐标系下）
        
        // 调用相机模型的liftProjective函数进行去畸变
        cam->liftProjective(a, b);
        
        // 将3D点投影到归一化平面（z=1平面），得到归一化坐标
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;  // 返回归一化坐标
}

// 计算特征点在归一化平面上的速度
// 参数：
//   ids: 特征点ID向量，用于建立ID与点的对应关系
//   pts: 当前帧归一化平面坐标点集
//   cur_id_pts: 当前帧ID->点位置的映射（输出参数）
//   prev_id_pts: 上一帧ID->点位置的映射（输入参数）
// 返回值：特征点速度向量（归一化平面上的速度，单位：m/s）
vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;  // 存储速度结果的向量
    cur_id_pts.clear();  // 清空当前帧ID映射，准备重新填充
    
    // 建立当前帧ID到点位置的映射
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        // 使用make_pair创建键值对并插入到map中
        // key: 特征点ID, value: 对应的归一化平面坐标
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // 计算点速度（如果有上一帧数据）
    if (!prev_id_pts.empty())  // 检查上一帧映射是否为空
    {
        double dt = cur_time - prev_time;  // 计算时间差（秒）
        
        // 遍历当前帧所有特征点
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;  // map迭代器
            
            // 在上一帧映射中查找当前特征点的ID
            it = prev_id_pts.find(ids[i]);
            
            // 如果找到了（特征点在上一帧也存在）
            if (it != prev_id_pts.end())
            {
                // 计算x方向速度：(当前x - 上一帧x) / 时间差
                double v_x = (pts[i].x - it->second.x) / dt;
                // 计算y方向速度：(当前y - 上一帧y) / 时间差
                double v_y = (pts[i].y - it->second.y) / dt;
                
                // 将速度向量添加到结果中
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                // 如果没找到（新特征点），速度设为0
                pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    else
    {
        // 如果没有上一帧数据（第一帧），所有速度设为0
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;  // 返回速度向量
}

// 在图像上绘制特征点的跟踪轨迹和相关信息
// 参数：
//   imLeft: 左目图像
//   imRight: 右目图像
//   curLeftIds: 当前左图特征点ID向量
//   curLeftPts: 当前左图特征点像素坐标
//   curRightPts: 当前右图特征点像素坐标
//   prevLeftPtsMap: 上一帧左图ID->点位置的映射
void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;  // 获取图像行数（被注释掉了）
    int cols = imLeft.cols;    // 获取图像列数（宽度）
    
    // 如果右图存在且是双目相机，将左右图像水平拼接
    if (!imRight.empty() && stereo_cam)
        cv::hconcat(imLeft, imRight, imTrack);  // 水平拼接函数
    else
        imTrack = imLeft.clone();  // 单目时只复制左图
    
    // 将灰度图转换为彩色图，以便用不同颜色绘制
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    // 绘制左图特征点（用圆形表示）
    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        // 计算颜色值，根据跟踪时间长短渐变（跟踪时间越长，颜色越红）
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        // 绘制圆形：圆心、半径2、颜色(R,G,B)、线宽2
        // 颜色：从紫色(255,0,255)渐变到红色(255,0,0)
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    
    // 绘制右图特征点（如果是双目）
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;  // 右图特征点x坐标加上左图宽度，因为左右图已拼接
            
            // 绘制右图特征点，用绿色表示
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            
            // 可选的：绘制左右匹配线（被注释掉了）
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    // 绘制跟踪轨迹（从上一帧到当前帧的箭头）
    map<int, cv::Point2f>::iterator mapIt;  // map迭代器
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];  // 当前特征点ID
        
        // 在上一帧映射中查找该ID
        mapIt = prevLeftPtsMap.find(id);
        
        // 如果找到（该特征点在上一帧也存在）
        if(mapIt != prevLeftPtsMap.end())
        {
            // 绘制箭头：从当前点指向上一帧点
            // 参数：图像、起点、终点、颜色(绿色)、线宽1、线型8、偏移0、箭头头部长度的比例0.2
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

    // 绘制预测点（用于调试，被注释掉了）
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    // 图像缩放（被注释掉了）
    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}

// 设置特征点的预测位置（通常来自IMU积分或运动模型）
// 参数：predictPts - 预测的3D点位置映射（ID->3D坐标）
void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
{
    hasPrediction = true;  // 设置预测标志为true
    predict_pts.clear();   // 清空预测点向量
    predict_pts_debug.clear();  // 清空调试预测点向量
    
    map<int, Eigen::Vector3d>::iterator itPredict;  // 预测点映射的迭代器
    
    // 遍历当前所有特征点ID
    for (size_t i = 0; i < ids.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = ids[i];  // 当前特征点ID
        
        // 在预测点映射中查找该ID
        itPredict = predictPts.find(id);
        
        // 如果找到了该ID的预测位置
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;  // 临时变量，存储投影后的像素坐标
            
            // 将3D点投影到图像平面（得到像素坐标）
            m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
            
            // 将预测的像素坐标添加到预测点向量中
            predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            // 如果没有该ID的预测，使用上一帧的位置作为预测
            predict_pts.push_back(prev_pts[i]);
    }
}

// 移除异常特征点
// 参数：removePtsIds - 需要移除的特征点ID集合
void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;  // set迭代器
    vector<uchar> status;  // 状态向量，1表示保留，0表示移除
    
    // 遍历所有特征点ID
    for (size_t i = 0; i < ids.size(); i++)
    {
        // 在需要移除的ID集合中查找当前ID
        itSet = removePtsIds.find(ids[i]);
        
        // 如果找到了（需要移除）
        if(itSet != removePtsIds.end())
            status.push_back(0);  // 标记为0（移除）
        else
            status.push_back(1);  // 标记为1（保留）
    }

    // 根据状态向量压缩各个容器
    reduceVector(prev_pts, status);  // 压缩上一帧点
    reduceVector(ids, status);       // 压缩ID向量
    reduceVector(track_cnt, status); // 压缩跟踪计数向量
}

// 获取跟踪图像（可视化结果）
cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;  // 返回绘制好的跟踪图像
}