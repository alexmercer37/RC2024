/*
 * @Author: Tommy0929
 * @Date: 2024-03-06 11:12:38
 * @LastEditors: Tommy0929 tommy07210728@163.com
 * @FilePath: /测完通信就可以上车的多线程代码，已封装（记得上车改英文名）/inc/opencv.h
 * WHUROBOCON_SAVED!!!
 */
#ifndef _OPENCVHEAD_H
#define _OPENCVHEAD_H
#include "OpencvHead.h"
#include <iostream>
#include <cstdio>
#include <string.h>
class cameraCV
{
public:
    void getContour(cv::Mat &input, cv::Mat &output);
    void detectStraightLine(cv::Mat &contour, std::vector<cv::Vec4f> &plines, cv::Mat &output);
    void getColor(const cv::Mat &input, cv::Mat &mask, cv::Mat &output, const int color_num[]);

private:
    cv::Mat hsv, gray, median, laplacian, lap8BitFrame;
};

#endif