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
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// CameraPoseVisualization类：用于可视化相机位姿的ROS工具类
// 功能：生成并发布用于在RViz中显示相机位姿和轨迹的Marker消息
class CameraPoseVisualization {
public:
    // 命名空间：用于在RViz中区分不同的Marker
    std::string m_marker_ns;

    // 构造函数：初始化颜色和参数
    // 参数：r,g,b,a - 默认颜色（RGBA，范围0-1）
    CameraPoseVisualization(float r, float g, float b, float a);
    
    // 设置相机图像边界颜色（用于绘制相机视锥）
    void setImageBoundaryColor(float r, float g, float b, float a=1.0);
    
    // 设置光心连接线颜色（连接相机位置和图像中心的线）
    void setOpticalCenterConnectorColor(float r, float g, float b, float a=1.0);
    
    // 设置相机可视化尺度（控制相机模型的大小）
    void setScale(double s);
    
    // 设置线条宽度
    void setLineWidth(double width);

    // 添加一个相机位姿到可视化中
    // 参数：
    //   p: 相机位置（世界坐标系）
    //   q: 相机旋转（四元数）
    void add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
    
    // 重置所有可视化标记（清空之前添加的内容）
    void reset();

    // 通过指定的发布器发布Marker消息
    // 参数：
    //   pub: ROS发布器
    //   header: ROS消息头（包含时间戳和坐标系信息）
    void publish_by(ros::Publisher& pub, const std_msgs::Header& header);
    
    // 添加一条普通边（用于连接两个相机位姿，表示轨迹）
    void add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
    
    // 添加一条回环边（通常用不同颜色表示回环检测结果）
    void add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
    
private:
    // 存储所有可视化标记的容器
    std::vector<visualization_msgs::Marker> m_markers;
    
    // 图像边界颜色（RGBA）
    std_msgs::ColorRGBA m_image_boundary_color;
    
    // 光心连接线颜色（RGBA）
    std_msgs::ColorRGBA m_optical_center_connector_color;
    
    // 可视化尺度
    double m_scale;
    
    // 线条宽度
    double m_line_width;

    // =========== 静态常量：预定义的相机模型关键点 ===========
    // 这些点定义了相机图像平面和光心的相对位置
    
    // 图像平面左下角（Image Left-Top）
    static const Eigen::Vector3d imlt;
    
    // 图像平面右下角（Image Left-Bottom）
    static const Eigen::Vector3d imlb;
    
    // 图像平面左上角（Image Right-Top）
    static const Eigen::Vector3d imrt;
    
    // 图像平面右上角（Image Right-Bottom）
    static const Eigen::Vector3d imrb;
    
    // 光心（Optical Center） - 相机中心点
    static const Eigen::Vector3d oc;
    
    // 相机坐标系下的三个辅助点，用于绘制相机箭头（表示方向）
    static const Eigen::Vector3d lt0;
    static const Eigen::Vector3d lt1;
    static const Eigen::Vector3d lt2;
};