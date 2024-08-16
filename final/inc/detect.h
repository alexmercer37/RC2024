/*
 * @Author: Tommy0929
 * @Date: 2024-03-06 11:12:38
 * @LastEditors: Tommy0929 tommy07210728@163.com
 * @FilePath: /final/inc/detect.h
 * WHUROBOCON_SAVED!!!
 */
#ifndef DETECT_H
#define DETECT_H

#include "camera.h"
#include "opencv.h"
#include "uart.h"
#include <sys/types.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "pointcloud.h"
#include <semaphore.h>
#include "OpencvHead.h"
#include "tensorRT/builder/trt_builder.hpp"
#include "application/app_yolo/yolo.hpp"
#include "application/app_yolo/multi_gpu.hpp"
typedef struct
{
    int imgWidth, imgHeight, i;
    ObjectDetector::Box *box_ptr;
    cv::Mat cv_rgb;
} args_ptr;

class detect
{
public:
    void detect_distance(const std::shared_ptr<Yolo::Infer> &yolo, cv::Mat &cv_rgb);
    void detect_distance_no_threead(ObjectDetector::BoxArray &bboxes, cv::Mat color, uint32_t &NUM);
    void dstance_pthread(ObjectDetector::BoxArray &bboxes, cv::Mat rgb, uint32_t &NUM);
    void detect_boxes(ObjectDetector::BoxArray &bboxes, cv::Mat rgb, uint32_t &NUM, cv::Mat &cv_depth, k4a::transformation &k4aTransformation, k4a::calibration &k4aCalibration, const int color_num[]);
    void time();
    float* distance(cv::Mat &rgb, int left, int right, int top, int bottom);
    float distance(float h);
    std::vector<float> findMinSecondElement(const std::vector<std::vector<float>> &group);
    void getMinData(std::vector<std::vector<float>> &values, float threshold);
    void detect_labels(ObjectDetector::BoxArray &bboxes, cv::Mat rgb, uint32_t &NUM);
    explicit detect()
        : contour(), capture(), yolo(Yolo::create_infer("red.trtmodel", Yolo::Type::V5, 0, 0.8f, 0.5f)) {}
    // static void *detect_purple_pthread_wrapper(void *arg)
    // {
    //     args_ptr *args = static_cast<args_ptr *>(arg);
    //     detect detectInstance;
    //     detectInstance.detect_purple_pthread(args);
    //     delete args;
    //     return nullptr;
    // }

private:
    const std::shared_ptr<Yolo::Infer> &yolo;
    cv::Mat contour;
    k4a::capture capture;
    std::shared_future<Yolo::BoxArray> prefuture;
};
#endif
