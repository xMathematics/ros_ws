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

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

// 命名空间声明
using namespace std;        // 使用标准C++库命名空间，方便使用vector、map等标准容器
using namespace camodocal;  // 使用camodocal相机标定库的命名空间
using namespace Eigen;      // 使用Eigen数学库命名空间，用于矩阵运算

// 函数声明
bool inBorder(const cv::Point2f &pt);  // 检查点是否在图像边界内的函数声明
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);  // 根据状态向量缩减点向量
void reduceVector(vector<int> &v, vector<uchar> status);           // 根据状态向量缩减ID向量

// 特征跟踪器类定义
class FeatureTracker
{
public:
    // 构造函数
    FeatureTracker();
    
    // 核心函数：处理图像并跟踪特征点
    // 参数：当前时间、当前图像、可选的右目图像（用于双目）
    // 返回值：映射结构，键为特征点ID，值为包含帧号和7维特征信息的向量对
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    
    // 设置特征提取的掩码，避免特征点过于聚集
    void setMask();
    
    // 从标定文件读取相机内参
    void readIntrinsicParameter(const vector<string> &calib_file);
    
    // 显示去畸变后的图像（用于调试）
    void showUndistortion(const string &name);
    
    // 使用基础矩阵F进行外点剔除
    void rejectWithF();
    
    // 对特征点进行去畸变处理
    void undistortedPoints();
    
    // 对给定点集进行去畸变
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    
    // 计算特征点的像素速度（当前帧与上一帧之间的位移）
    // 参数：ID列表、当前点、当前ID到点的映射、上一帧ID到点的映射
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    
    // 显示两幅图像及其上的特征点（用于调试）
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    
    // 在图像上绘制特征点的跟踪轨迹
    // 参数：左右图像、当前左图特征点ID、当前左图特征点、当前右图特征点、上一帧左图特征点映射
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    
    // 设置特征点的预测位置（可能来自IMU或运动模型）
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    
    // 计算两个像素点之间的欧氏距离
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    
    // 移除异常特征点
    void removeOutliers(set<int> &removePtsIds);
    
    // 获取跟踪可视化图像
    cv::Mat getTrackImage();
    
    // 检查点是否在图像边界内（类成员版本）
    bool inBorder(const cv::Point2f &pt);
    
    // 成员变量

    int row, col;           // 图像的行数和列数（高和宽）
    
    // OpenCV图像变量
    cv::Mat imTrack;        // 用于绘制跟踪结果的图像
    cv::Mat mask;           // 特征提取掩码，避免特征点聚集
    cv::Mat fisheye_mask;   // 鱼眼相机掩码，用于去除边缘畸变严重的区域
    cv::Mat prev_img, cur_img;  // 上一帧和当前帧图像
    
    // 特征点容器
    vector<cv::Point2f> n_pts;           // 新提取的特征点
    vector<cv::Point2f> predict_pts;     // 预测的特征点位置
    vector<cv::Point2f> predict_pts_debug; // 用于调试的预测点
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;  // 上一帧、当前帧左图、当前帧右图的像素点
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;  // 去畸变后的对应点
    
    // 特征点速度
    vector<cv::Point2f> pts_velocity, right_pts_velocity;  // 左右图特征点像素速度
    
    // 特征点ID和跟踪计数
    vector<int> ids, ids_right;      // 左右图特征点的唯一ID
    vector<int> track_cnt;           // 每个特征点被跟踪的帧数
    
    // 映射结构：ID -> 点位置，便于快速查找
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;           // 当前帧和上一帧去畸变点映射
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map; // 右图对应映射
    map<int, cv::Point2f> prevLeftPtsMap;  // 上一帧左图点映射
    
    // 相机模型
    vector<camodocal::CameraPtr> m_camera;  // 相机模型指针（支持多个相机，如双目）
    
    // 时间相关
    double cur_time;    // 当前帧时间戳
    double prev_time;   // 上一帧时间戳
    
    // 配置和状态标志
    bool stereo_cam;    // 是否为双目相机系统
    int n_id;           // 下一个可用的特征点ID（自增计数器）
    bool hasPrediction; // 是否有预测信息可用
};