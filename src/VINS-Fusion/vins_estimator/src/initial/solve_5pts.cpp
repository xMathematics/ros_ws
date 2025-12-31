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

#include "solve_5pts.h"


// 分解本质矩阵E，得到4个可能的相机姿态
// 参数说明：
// - _E: 输入的3x3本质矩阵
// - _R1, _R2: 输出的两个可能的旋转矩阵
// - _t: 输出的一个平移向量（另一个平移是-t）
namespace cv {
    void decomposeEssentialMat( InputArray _E, OutputArray _R1, OutputArray _R2, OutputArray _t )
    {
        // 获取本质矩阵E，并确保是3x3矩阵
        Mat E = _E.getMat().reshape(1, 3);  // reshape为单通道3x3矩阵
        CV_Assert(E.cols == 3 && E.rows == 3);  // 断言确保尺寸正确

        // 对本质矩阵E进行奇异值分解(SVD)：E = U * diag(1,1,0) * V^T
        Mat D, U, Vt;
        SVD::compute(E, D, U, Vt);  // U和Vt是正交矩阵

        // 确保U和V的行列式为+1（保持右手坐标系）
        // 如果行列式为负，则乘以-1
        if (determinant(U) < 0) U *= -1.;
        if (determinant(Vt) < 0) Vt *= -1.;

        // 创建旋转矩阵W（90度绕Z轴旋转）
        Mat W = (Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
        W.convertTo(W, E.type());  // 转换为与E相同的数据类型

        // 计算两个可能的旋转矩阵
        // 根据Hartley和Zisserman的《Multiple View Geometry》中的公式：
        // R1 = U * W * V^T, R2 = U * W^T * V^T
        Mat R1, R2, t;
        R1 = U * W * Vt;    // 第一个旋转矩阵
        R2 = U * W.t() * Vt; // 第二个旋转矩阵（W转置）
        
        // 平移向量：取U的第三列（对应零奇异值的奇异向量）
        t = U.col(2) * 1.0;  // 尺度未知，取单位向量

        // 输出结果
        R1.copyTo(_R1);  // 复制第一个旋转矩阵到输出
        R2.copyTo(_R2);  // 复制第二个旋转矩阵到输出
        t.copyTo(_t);    // 复制平移向量到输出
    }

       // 从本质矩阵E中恢复相机姿态
    // 参数说明：
    // - E: 本质矩阵
    // - _points1, _points2: 两幅图像中的匹配点对
    // - _cameraMatrix: 相机内参矩阵
    // - _R, _t: 输出的旋转矩阵和平移向量
    // - _mask: 输入输出的掩码（内点/外点标记）
    // 返回值：内点数量
    int recoverPose( InputArray E, InputArray _points1, InputArray _points2, InputArray _cameraMatrix,
                         OutputArray _R, OutputArray _t, InputOutputArray _mask)
    {
        // 将输入转换为double类型的Mat
        Mat points1, points2, cameraMatrix;
        _points1.getMat().convertTo(points1, CV_64F);  // 转换为double
        _points2.getMat().convertTo(points2, CV_64F);
        _cameraMatrix.getMat().convertTo(cameraMatrix, CV_64F);

        // 检查点对数量是否匹配
        int npoints = points1.checkVector(2);  // 获取点的数量（每个点2维）
        CV_Assert( npoints >= 0 && points2.checkVector(2) == npoints &&
                                  points1.type() == points2.type());

        // 检查相机内参矩阵是否正确
        CV_Assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3 && cameraMatrix.channels() == 1);

        // 如果点是多通道的，重塑为单通道
        if (points1.channels() > 1)
        {
            points1 = points1.reshape(1, npoints);  // 重塑为npoints行，1通道
            points2 = points2.reshape(1, npoints);
        }

        // 获取相机内参
        double fx = cameraMatrix.at<double>(0,0);  // 焦距x
        double fy = cameraMatrix.at<double>(1,1);  // 焦距y
        double cx = cameraMatrix.at<double>(0,2);  // 主点x
        double cy = cameraMatrix.at<double>(1,2);  // 主点y

        // 将像素坐标转换为归一化平面坐标（去畸变）
        points1.col(0) = (points1.col(0) - cx) / fx;  // u = (u' - cx)/fx
        points2.col(0) = (points2.col(0) - cx) / fx;
        points1.col(1) = (points1.col(1) - cy) / fy;  // v = (v' - cy)/fy
        points2.col(1) = (points2.col(1) - cy) / fy;

        // 转置矩阵，使每列对应一个点（3行×n列）
        points1 = points1.t();
        points2 = points2.t();

        // 分解本质矩阵得到4个可能的解
        Mat R1, R2, t;
        decomposeEssentialMat(E, R1, R2, t);  // 得到R1, R2和t

        // 构建4个可能的相机姿态矩阵P = [R|t]
        Mat P0 = Mat::eye(3, 4, R1.type());  // 第一帧相机：单位矩阵（世界坐标系）
        
        Mat P1(3, 4, R1.type()), P2(3, 4, R1.type()), 
            P3(3, 4, R1.type()), P4(3, 4, R1.type());
        
        // P1 = [R1 | t]
        P1(Range::all(), Range(0, 3)) = R1 * 1.0;  // 前三列是旋转
        P1.col(3) = t * 1.0;  // 第四列是平移
        
        // P2 = [R2 | t]
        P2(Range::all(), Range(0, 3)) = R2 * 1.0;
        P2.col(3) = t * 1.0;
        
        // P3 = [R1 | -t]
        P3(Range::all(), Range(0, 3)) = R1 * 1.0;
        P3.col(3) = -t * 1.0;
        
        // P4 = [R2 | -t]
        P4(Range::all(), Range(0, 3)) = R2 * 1.0;
        P4.col(3) = -t * 1.0;

        // 进行cheirality检查（正深度检查）
        // 使用阈值dist过滤远处的点（可能是无穷远点）
        double dist = 50.0;  // 距离阈值
        Mat Q;  // 三角化后的3D点（齐次坐标）

        // 检查第一个解：P0和P1
        triangulatePoints(P0, P1, points1, points2, Q);  // 三角化得到3D点
        Mat mask1 = Q.row(2).mul(Q.row(3)) > 0;  // 检查深度符号（Z*W > 0）
        
        // 将齐次坐标转换为欧氏坐标（除以W）
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        
        mask1 = (Q.row(2) < dist) & mask1;  // 深度小于阈值
        Q = P1 * Q;  // 将点变换到第二个相机坐标系
        mask1 = (Q.row(2) > 0) & mask1;  // 在第二个相机前（正深度）
        mask1 = (Q.row(2) < dist) & mask1;  // 深度小于阈值

        // 检查第二个解：P0和P2
        triangulatePoints(P0, P2, points1, points2, Q);
        Mat mask2 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask2 = (Q.row(2) < dist) & mask2;
        Q = P2 * Q;
        mask2 = (Q.row(2) > 0) & mask2;
        mask2 = (Q.row(2) < dist) & mask2;

        // 检查第三个解：P0和P3
        triangulatePoints(P0, P3, points1, points2, Q);
        Mat mask3 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask3 = (Q.row(2) < dist) & mask3;
        Q = P3 * Q;
        mask3 = (Q.row(2) > 0) & mask3;
        mask3 = (Q.row(2) < dist) & mask3;

        // 检查第四个解：P0和P4
        triangulatePoints(P0, P4, points1, points2, Q);
        Mat mask4 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask4 = (Q.row(2) < dist) & mask4;
        Q = P4 * Q;
        mask4 = (Q.row(2) > 0) & mask4;
        mask4 = (Q.row(2) < dist) & mask4;

        // 将掩码转置回原来的方向
        mask1 = mask1.t();
        mask2 = mask2.t();
        mask3 = mask3.t();
        mask4 = mask4.t();

        // 如果提供了_mask，则用它过滤异常值
        if (!_mask.empty())
        {
            Mat mask = _mask.getMat();
            CV_Assert(mask.size() == mask1.size());
            bitwise_and(mask, mask1, mask1);  // 与输入掩码取与
            bitwise_and(mask, mask2, mask2);
            bitwise_and(mask, mask3, mask3);
            bitwise_and(mask, mask4, mask4);
        }
        
        // 如果需要_mask但未提供，则创建
        if (_mask.empty() && _mask.needed())
        {
            _mask.create(mask1.size(), CV_8U);
        }

        // 确保输出_R和_t已分配空间
        CV_Assert(_R.needed() && _t.needed());
        _R.create(3, 3, R1.type());
        _t.create(3, 1, t.type());

        // 统计每个解的内点数量
        int good1 = countNonZero(mask1);
        int good2 = countNonZero(mask2);
        int good3 = countNonZero(mask3);
        int good4 = countNonZero(mask4);

        // 选择内点数量最多的解作为最终解
        if (good1 >= good2 && good1 >= good3 && good1 >= good4)
        {
            R1.copyTo(_R);  // 输出旋转矩阵R1
            t.copyTo(_t);   // 输出平移向量t
            if (_mask.needed()) mask1.copyTo(_mask);  // 输出内点掩码
            return good1;   // 返回内点数量
        }
        else if (good2 >= good1 && good2 >= good3 && good2 >= good4)
        {
            R2.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed()) mask2.copyTo(_mask);
            return good2;
        }
        else if (good3 >= good1 && good3 >= good2 && good3 >= good4)
        {
            t = -t;  // 需要翻转平移方向
            R1.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed()) mask3.copyTo(_mask);
            return good3;
        }
        else  // good4最大
        {
            t = -t;  // 需要翻转平移方向
            R2.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed()) mask4.copyTo(_mask);
            return good4;
        }
    }
    // recoverPose的简化版本，使用焦距和主点而不是完整的相机矩阵
    int recoverPose( InputArray E, InputArray _points1, InputArray _points2, OutputArray _R,
                         OutputArray _t, double focal, Point2d pp, InputOutputArray _mask)
    {
        // 根据焦距和主点构造相机内参矩阵
        Mat cameraMatrix = (Mat_<double>(3,3) << focal, 0, pp.x,  // fx, 0, cx
                                                 0, focal, pp.y,   // 0, fy, cy
                                                 0, 0, 1);         // 0, 0, 1
        // 调用完整版本的recoverPose
        return cv::recoverPose(E, _points1, _points2, cameraMatrix, _R, _t, _mask);
    }
}

// 从匹配点对计算两视图之间的相对位姿（旋转和平移）
// 参数说明：
// - corres: 匹配点对向量，每个元素是一对归一化坐标（或像素坐标）
// - Rotation: 输出的旋转矩阵（从第二帧到第一帧）
// - Translation: 输出的平移向量（从第二帧到第一帧）
// 返回值：是否成功计算相对位姿
bool MotionEstimator::solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, 
                                      Matrix3d &Rotation, Vector3d &Translation)
{
    // 需要至少15对匹配点（5点法最小需要5对，但这里使用RANSAC需要更多点）
    if (corres.size() >= 15)
    {
        // 准备OpenCV格式的点集
        vector<cv::Point2f> ll, rr;  // ll: 左图（第一帧）点，rr: 右图（第二帧）点
        for (int i = 0; i < int(corres.size()); i++)
        {
            // 将Eigen::Vector3d转换为cv::Point2f（忽略z坐标）
            // 注意：这里假设输入的是归一化坐标，z=1
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
            rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
        }
        
        cv::Mat mask;  // 内点掩码
        // 使用RANSAC方法计算基础矩阵F
        // 参数说明：
        // - ll, rr: 匹配点对
        // - cv::FM_RANSAC: 使用RANSAC方法
        // - 0.3 / 460: RANSAC阈值（像素误差），这里假设焦距大约为460
        // - 0.99: 置信度
        // - mask: 输出的内点掩码
        cv::Mat E = cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 0.3 / 460, 0.99, mask);
        
        // 创建单位内参矩阵（假设使用归一化坐标）
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 
                                                           0, 1, 0, 
                                                           0, 0, 1);
        
        cv::Mat rot, trans;  // OpenCV格式的旋转矩阵和平移向量
        // 从本质矩阵E恢复相机姿态
        // 注意：这里的E实际上是本质矩阵（当使用归一化坐标时，F=E）
        int inlier_cnt = cv::recoverPose(E, ll, rr, cameraMatrix, rot, trans, mask);
        //cout << "inlier_cnt " << inlier_cnt << endl;

        // 将OpenCV结果转换为Eigen格式
        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        for (int i = 0; i < 3; i++)
        {   
            T(i) = trans.at<double>(i, 0);  // 提取平移向量
            for (int j = 0; j < 3; j++)
                R(i, j) = rot.at<double>(i, j);  // 提取旋转矩阵
        }

        // 坐标转换：将相机姿态从第二帧坐标系转换到第一帧坐标系
        // OpenCV的recoverPose返回的是从第一帧到第二帧的变换
        // 我们需要的是从第二帧到第一帧的变换
        Rotation = R.transpose();  // 旋转的逆
        Translation = -R.transpose() * T;  // 平移的逆
        
        // 判断是否成功：内点数量大于12
        if(inlier_cnt > 12)
            return true;
        else
            return false;
    }
    return false;  // 匹配点太少，返回失败
}