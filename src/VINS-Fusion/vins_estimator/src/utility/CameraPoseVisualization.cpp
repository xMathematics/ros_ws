/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "CameraPoseVisualization.h"

// ==================== 静态常量定义 ====================
// 这些常量定义了相机模型在相机坐标系（光心为原点，Z轴为光轴）中的关键点坐标
// 注意：这里的坐标假设图像平面在Z=1处（即焦距为1），相机光心在(0,0,0)

// 图像平面左上角点（Image Left-Top）：(-1.0, -0.5, 1.0)
// 解释：在相机坐标系中，图像左上角在x=-1.0（左），y=-0.5（下），z=1.0（前方）
// 通常图像坐标系的x轴向右，y轴向下，这里使用不同的约定
const Eigen::Vector3d CameraPoseVisualization::imlt = Eigen::Vector3d(-1.0, -0.5, 1.0);

// 图像平面右上角点（Image Right-Top）：(1.0, -0.5, 1.0)
const Eigen::Vector3d CameraPoseVisualization::imrt = Eigen::Vector3d( 1.0, -0.5, 1.0);

// 图像平面左下角点（Image Left-Bottom）：(-1.0, 0.5, 1.0)
const Eigen::Vector3d CameraPoseVisualization::imlb = Eigen::Vector3d(-1.0,  0.5, 1.0);

// 图像平面右下角点（Image Right-Bottom）：(1.0, 0.5, 1.0)
const Eigen::Vector3d CameraPoseVisualization::imrb = Eigen::Vector3d( 1.0,  0.5, 1.0);

// 用于绘制相机方向箭头的三个点（在图像平面左上角区域）
const Eigen::Vector3d CameraPoseVisualization::lt0 = Eigen::Vector3d(-0.7, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt1 = Eigen::Vector3d(-0.7, -0.2, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt2 = Eigen::Vector3d(-1.0, -0.2, 1.0);

// 光心（Optical Center）：(0.0, 0.0, 0.0)
const Eigen::Vector3d CameraPoseVisualization::oc = Eigen::Vector3d(0.0, 0.0, 0.0);

// ==================== 辅助函数 ====================
// 将Eigen::Vector3d转换为geometry_msgs::Point
void Eigen2Point(const Eigen::Vector3d& v, geometry_msgs::Point& p) {
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
}

// ==================== 构造函数 ====================
CameraPoseVisualization::CameraPoseVisualization(float r, float g, float b, float a)
    : m_marker_ns("CameraPoseVisualization"),  // 默认命名空间
      m_scale(0.2),                            // 默认缩放因子
      m_line_width(0.01) {                     // 默认线宽
    // 设置图像边界颜色（使用传入的RGBA值）
    m_image_boundary_color.r = r;
    m_image_boundary_color.g = g;
    m_image_boundary_color.b = b;
    m_image_boundary_color.a = a;
    
    // 设置光心连接线颜色（初始与图像边界颜色相同）
    m_optical_center_connector_color.r = r;
    m_optical_center_connector_color.g = g;
    m_optical_center_connector_color.b = b;
    m_optical_center_connector_color.a = a;
}

// ==================== 设置颜色和样式 ====================
void CameraPoseVisualization::setImageBoundaryColor(float r, float g, float b, float a) {
    m_image_boundary_color.r = r;
    m_image_boundary_color.g = g;
    m_image_boundary_color.b = b;
    m_image_boundary_color.a = a;
}

void CameraPoseVisualization::setOpticalCenterConnectorColor(float r, float g, float b, float a) {
    m_optical_center_connector_color.r = r;
    m_optical_center_connector_color.g = g;
    m_optical_center_connector_color.b = b;
    m_optical_center_connector_color.a = a;
}

void CameraPoseVisualization::setScale(double s) {
    m_scale = s;  // 设置相机模型的缩放因子
}

void CameraPoseVisualization::setLineWidth(double width) {
    m_line_width = width;  // 设置线宽
}

// ==================== 添加边（轨迹线） ====================
// 添加一条普通边，用于连接两个相机位置（通常表示轨迹）
void CameraPoseVisualization::add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1){
    // 创建一个线条列表（LINE_LIST）类型的Marker
    // LINE_LIST：每两个点构成一条独立的线段
    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;                          // 命名空间
    marker.id = m_markers.size() + 1;                // 唯一的ID（使用当前标记数+1）
    marker.type = visualization_msgs::Marker::LINE_LIST;  // 线条列表类型
    marker.action = visualization_msgs::Marker::ADD;     // 添加动作
    marker.scale.x = 0.005;                           // 线宽（这里固定为0.005）

    // 设置颜色为绿色（RGB: 0,1,0,1）
    marker.color.g = 1.0f;
    marker.color.a = 1.0;

    // 将两个三维点从Eigen格式转换为ROS消息格式
    geometry_msgs::Point point0, point1;
    Eigen2Point(p0, point0);
    Eigen2Point(p1, point1);

    // 将点添加到Marker中（两点构成一条线段）
    marker.points.push_back(point0);
    marker.points.push_back(point1);

    // 将Marker添加到列表中
    m_markers.push_back(marker);
}

// ==================== 添加回环边 ====================
// 添加一条回环边，通常用特殊颜色（洋红色）和更粗的线表示
void CameraPoseVisualization::add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1){
    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;
    marker.id = m_markers.size() + 1;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.04;  // 回环边更粗，便于识别
    //marker.scale.x = 0.3;  // 注释掉的备用线宽

    // 设置颜色为洋红色（红+蓝）：RGB(1,0,1,1)
    marker.color.r = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    geometry_msgs::Point point0, point1;
    Eigen2Point(p0, point0);
    Eigen2Point(p1, point1);

    marker.points.push_back(point0);
    marker.points.push_back(point1);

    m_markers.push_back(marker);
}

// ==================== 添加相机位姿 ====================
// 主要函数：添加一个完整的相机模型可视化
void CameraPoseVisualization::add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    // 创建一个线条条带（LINE_STRIP）类型的Marker
    // LINE_STRIP：所有点依次连接形成连续的折线
    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;
    marker.id = m_markers.size() + 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = m_line_width;  // 使用设置的线宽

    // 设置Marker的位姿为单位变换
    // 注意：Marker本身的位姿是单位变换，所有点的坐标已经通过q和p变换到世界坐标系
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    // 转换所有预定义的点到世界坐标系
    // 步骤：1. 缩放 2. 旋转 3. 平移
    geometry_msgs::Point pt_lt, pt_lb, pt_rt, pt_rb, pt_oc, pt_lt0, pt_lt1, pt_lt2;

    // 图像平面四个角点
    Eigen2Point(q * (m_scale * imlt) + p, pt_lt);  // 左上角
    Eigen2Point(q * (m_scale * imlb) + p, pt_lb);  // 左下角
    Eigen2Point(q * (m_scale * imrt) + p, pt_rt);  // 右上角
    Eigen2Point(q * (m_scale * imrb) + p, pt_rb);  // 右下角
    
    // 方向箭头的三个点
    Eigen2Point(q * (m_scale * lt0) + p, pt_lt0);
    Eigen2Point(q * (m_scale * lt1) + p, pt_lt1);
    Eigen2Point(q * (m_scale * lt2) + p, pt_lt2);
    
    // 光心点
    Eigen2Point(q * (m_scale * oc) + p, pt_oc);

    // ===== 构建相机模型线条 =====
    
    // 1. 图像边界（四个边）
    // 左上 -> 左下
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_lb);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);
    
    // 左下 -> 右下
    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_rb);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);
    
    // 右下 -> 右上
    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_rt);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);
    
    // 右上 -> 左上
    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_lt);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    // 2. 左上角方向箭头（用于指示相机朝向）
    // 箭头由两条短线组成：lt0->lt1, lt1->lt2
    marker.points.push_back(pt_lt0);
    marker.points.push_back(pt_lt1);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_lt1);
    marker.points.push_back(pt_lt2);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    // 3. 光心连接线（从图像平面四个角点到光心，形成视锥）
    // 左上角 -> 光心
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);
    
    // 左下角 -> 光心
    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);
    
    // 右上角 -> 光心
    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);
    
    // 右下角 -> 光心
    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    // 将完整的相机模型Marker添加到列表
    m_markers.push_back(marker);
}

// ==================== 重置 ====================
// 清空所有存储的Marker
void CameraPoseVisualization::reset() {
    m_markers.clear();
}

// ==================== 发布函数 ====================
// 将所有Marker打包成MarkerArray并通过指定的发布器发布
void CameraPoseVisualization::publish_by(ros::Publisher &pub, const std_msgs::Header &header) {
    visualization_msgs::MarkerArray markerArray_msg;
    
    // 为每个Marker设置消息头（时间戳和坐标系）
    for(auto& marker : m_markers) {
        marker.header = header;
        markerArray_msg.markers.push_back(marker);
    }

    // 发布Marker数组
    pub.publish(markerArray_msg);
}