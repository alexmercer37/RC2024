/*
 * @Author: ddxy
 * @Date: 2023-10-10 10:43:39
 * @LastEditors: Tommy0929 tommy07210728@163.com
 * @FilePath: /final/src/opencv.cpp
 * WHUROBOCON_SAVED!!!
 */ \
#include "../inc/opencv.h"

void cameraCV::getContour(cv::Mat &input, cv::Mat &output)
{
	cv::medianBlur(input, median, 3);
	cv::cvtColor(median, gray, cv::COLOR_BGRA2GRAY);
	cv::Laplacian(gray, laplacian, 3, 3);				// laplacian算子提取轮廓
	cv::convertScaleAbs(laplacian, lap8BitFrame);		// 将图片压缩为8位
	cv::Canny(lap8BitFrame, output, 50, 150, 3, false); // canny提取轮廓
}

void cameraCV::getColor(const cv::Mat &input, cv::Mat &mask, cv::Mat &output, const int color_num[])
{
	mask = cv::Mat::zeros(input.size(), CV_8UC1);
	cv::cvtColor(input, output, cv::COLOR_BGR2HSV);
	cv::inRange(output, cv::Scalar(color_num[0], color_num[1], color_num[2]), cv::Scalar(color_num[3], color_num[4], color_num[5]), mask);
	//cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	//cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
	//cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
        //input.copyTo(output, mask);
        // cv::imshow("color_num[0]", output);
	// cv::waitKey(1);
}

void cameraCV::detectStraightLine(cv::Mat &contour, std::vector<cv::Vec4f> &plines, cv::Mat &output)
{
	cv::HoughLinesP(contour, plines, 1, CV_PI / 180, 300, 200, 40); // 霍夫直线检测，参数：8位单通道图像，cv::Vec4f,rho,theta,最小的直线长度，两段直线认为是一根直线的最小的距离
	for (size_t i = 0; i < plines.size(); i++)
	{
		cv::Vec4f hlines = plines[i];
		cv::line(output, cv::Point(hlines[0], hlines[1]), cv::Point(hlines[2], hlines[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
	}
}
