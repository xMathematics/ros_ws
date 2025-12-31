/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

// TicToc类：用于代码执行时间测量和性能分析的工具类
// 用途：通常用于VIO/SLAM系统中各模块的性能分析和调试
class TicToc
{
  public:
    // 构造函数：创建对象时自动调用tic()开始计时
    TicToc()
    {
        tic();  // 对象创建时自动开始计时
    }

    // 开始计时函数：记录当前时间点作为起始时间
    void tic()
    {
        // 获取当前系统时间点
        // std::chrono::system_clock::now() 返回当前时间的时间点
        start = std::chrono::system_clock::now();
    }

    // 结束计时函数：计算从tic()到toc()经过的时间
    // 返回值：经过的时间，以毫秒(ms)为单位
    double toc()
    {
        // 获取当前系统时间点作为结束时间
        end = std::chrono::system_clock::now();
        
        // 计算时间间隔：end - start
        // std::chrono::duration<double> 表示以秒为单位的双精度时间间隔
        std::chrono::duration<double> elapsed_seconds = end - start;
        
        // 将时间间隔从秒转换为毫秒，并返回
        // count()返回时间间隔的秒数，乘以1000转换为毫秒
        return elapsed_seconds.count() * 1000;
    }

  private:
    // 私有成员变量：存储起始和结束时间点
    std::chrono::time_point<std::chrono::system_clock> start, end;
    // std::chrono::time_point: 表示时间线上的一个特定点
    // std::chrono::system_clock: 系统范围的实时时钟，可以转换为日历时间
};