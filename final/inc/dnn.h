// #ifndef DNN_H
// #define DNN_H
// #include <string>
// #include <fstream>
// #include <sstream>
// #include <iostream>
// #include <opencv2/dnn.hpp>     // 深度学习模块
// #include <opencv2/imgproc.hpp> // 图像处理模块
// #include <opencv2/highgui.hpp> // GUI图形

// #include <time.h>
// using namespace std;
// // 自定义配置结构
// struct Configuration
// {
// public:
//     float confThreshold; // Confidence threshold,置信度*分类分数后的阈值
//     float nmsThreshold;  // Non-maximum suppression threshold,iou阈值
//     float objThreshold;  // Object Confidence threshold,置信度阈值
//     string modelpath;    // 模型路径
// };

// // 模型
// class YOLOv5
// {
// public:
//     // 初始化
//     YOLOv5(Configuration config, bool isCuda);
//     void detect(cv::Mat &frame); // 检测函数
// private:
//     // float* anchors;
//     // int num_stride;
//     float confThreshold;
//     float nmsThreshold;
//     float objThreshold;
//     int inpWidth;
//     int inpHeight;
//     int num_classes;
//     string classes[80] = {"red_ball"};
//     cv::dnn::Net net;
//     // vector<string> class_names;
//     const bool keep_ratio = true; // 保持长宽比进行缩放                    // dnn里的
//     void drawPred(float conf, int left, int top, int right, int bottom, cv::Mat &frame, int classid);
//     cv::Mat resize_image(cv::Mat srcimg, int *newh, int *neww, int *top, int *left);
// };

// #endif