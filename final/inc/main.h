/*
 * @Author: Tommy0929
 * @Date: 2024-01-24 22:35:20
 * @LastEditors: Tommy0929 tommy07210728@163.com
 * @FilePath: /final/inc/main.h
 * WHUROBOCON_SAVED!!!
 */
#pragma once
#include <pthread.h>
#include "uart.h"
#include "camera.h"
#include "opencv.h"
#include "detect.h"
#include "pointcloud.h"

pthread_mutex_t protect_picture;
pthread_mutex_t buff_mutex;
pthread_mutex_t infer_mutex;

cv::Mat rgb_frame, depth_frame, *rgb_ptr, *depth_ptr;
cv::Mat *rgb_ptr2 = new cv::Mat;
k4a::capture capture;
k4a::device Device;
k4a::transformation k4aTransformation;
k4a::calibration k4aCalibration;

std::shared_future<Yolo::BoxArray> prefuture;
std::shared_ptr<cv::Mat> matBuff;
std::shared_ptr<cv::Mat> matBuff2;
std::shared_ptr<cv::Mat> matBuff3;
std::shared_ptr<cv::Mat> depthBuff;
std::shared_ptr<cv::Mat> depthBuff2;

camera *camera_ptr;
detect *Detect;

class pthread
{
public:
    void *k4aUpdate(void *camera_ptr);
    void *picture(void *argc);
    void *picture2(void *argc);
    void *create_infer(void *argc);
    void *infer_labels(void *argc);

private:
    camera *camera_ptr;
};

void getColor(const cv::Mat &input, cv::Mat &mask)
{
    mask = cv::Mat::zeros(input.size(), CV_8UC1);
    cv::cvtColor(input, input, cv::COLOR_BGR2HSV);
    cv::inRange(input, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), mask);

    input.copyTo(input, mask);
    cv::imshow("mask", mask);

    int white = cv::countNonZero(mask);
    int total = mask.total();

    if (white > total * 0.6)
    {
      cout << "ok" << endl;
    }
    else
    {  
      cv::inRange(input, cv::Scalar(125, 43, 46), cv::Scalar(155, 255, 255), mask);
      input.copyTo(input, mask);
      cv::imshow("mask", mask);
      int purple = cv::countNonZero(mask);
      int all = mask.total();
      if (purple > all * 0.6)
      {
        cout<< "error" <<endl;
      }
    }


}

void *k4aUpdate(void *camera_ptr)
{
    camera *Camera = (camera *)camera_ptr;

    while (1)
    {

        Camera->picture_update(Device, capture);
        
        pthread_mutex_lock(&buff_mutex);

        rgb_ptr = Camera->getpicture(Device, capture, rgb_frame, k4aTransformation);
        //rgb_ptr2 = Camera->getpicture(Device, capture, rgb_frame, k4aTransformation);
        depth_ptr = Camera->getdepth(Device, capture, depth_frame, k4aTransformation);

       // Camera->getAngel(Device);

        pthread_mutex_unlock(&buff_mutex);
        

        cv::cvtColor(*rgb_ptr, *rgb_ptr, cv::COLOR_BGRA2BGR);


        pthread_mutex_lock(&buff_mutex);

        //matBuff = std::make_shared<cv::Mat>(rgb_ptr->clone());
        matBuff2 = std::make_shared<cv::Mat>(rgb_ptr->clone());
        //matBuff3 = std::make_shared<cv::Mat>(rgb_ptr->clone());
        depthBuff = std::make_shared<cv::Mat>(depth_ptr->clone());
        //depthBuff2 = std::make_shared<cv::Mat>(depth_ptr->clone());

        pthread_mutex_unlock(&buff_mutex);
        

        //if (matBuff->empty() || matBuff2->empty() || depthBuff->empty())
        //{
        //    cout << "error" << endl;
        //}

        rgb_ptr->release();
        //rgb_ptr2->release();
        depth_ptr->release();
        
        usleep(100);   
    }
    pthread_exit(NULL);
}

void *init_usb_camera(void *argc)
{
  cv::VideoCapture capture(2);
  while (true)
  {
    cv::Mat frame;
    capture.read(frame);
    
    //matBuff2 = std::make_shared<cv::Mat>(frame.clone());
    
    imshow("video", frame);
    cv::waitKey(1); 

    //cv::Rect select;
    //select = cv::Rect(440, 160, 400, 400);
    //cv::Mat mask;
    //getColor(frame(select), mask);

  }
}

void *create_infer(void *argc)
{
    auto yoloEngine = Yolo::create_infer("redbest.trtmodel", Yolo::Type::V5, 0, 0.8f, 0.5f);
    while (1)
    {    
            
        if (prefuture.valid())
        {

            pthread_mutex_lock(&protect_picture);
            pthread_mutex_lock(&buff_mutex);
        
            prefuture = yoloEngine->commit(*matBuff2);
            auto bboxes = prefuture.get();

            pthread_mutex_unlock(&buff_mutex);
            usleep(100);

            int color_num[7] = {156, 43, 46, 180, 255, 255, 0x02};

            uint32_t NUM = bboxes.size();

            pthread_mutex_lock(&buff_mutex);

            for (auto box : bboxes)
            {
                Detect->detect_boxes(bboxes, *matBuff2, NUM, *depthBuff, k4aTransformation, k4aCalibration, color_num);
                // cout << "Red success" << endl;
            }

            pthread_mutex_unlock(&buff_mutex);
            usleep(100);
        }
        else
        {
            prefuture = yoloEngine->commit(*matBuff2);
        }

        pthread_mutex_lock(&buff_mutex);

        cv::imshow("red", *matBuff2);
        cv::waitKey(1);

        pthread_mutex_unlock(&buff_mutex);
        pthread_mutex_unlock(&protect_picture);

        usleep(100);
    }
    pthread_exit(NULL);
}

void *blue_infer(void *argc)
{
    auto yoloEngine = Yolo::create_infer("blue.trtmodel", Yolo::Type::V5, 0, 0.8f, 0.5f);
    while (1)
    {

        if (prefuture.valid())
        {

            pthread_mutex_lock(&protect_picture);
            pthread_mutex_lock(&buff_mutex);

            prefuture = yoloEngine->commit(*matBuff2);
            auto bboxes = prefuture.get();

            pthread_mutex_unlock(&buff_mutex);
            usleep(100);

            int color_num[7] = {100, 43, 46, 124, 255, 255, 0x02};

            uint32_t NUM = bboxes.size();

            pthread_mutex_lock(&buff_mutex);

            for (auto box : bboxes)
            {
                Detect->detect_boxes(bboxes, *matBuff2, NUM, *depthBuff, k4aTransformation, k4aCalibration, color_num);
                // cout << "Blue success" << endl;
            }

            pthread_mutex_unlock(&buff_mutex);
            usleep(100);
        }
        else
        {
            prefuture = yoloEngine->commit(*matBuff2);
        }

         //pthread_mutex_lock(&buff_mutex);

         //cv::imshow("blue", *matBuff2);
         //cv::waitKey(1);

         //pthread_mutex_unlock(&buff_mutex);
        pthread_mutex_unlock(&protect_picture);

        usleep(100);
    }
    pthread_exit(NULL);
}

void *picture(void *argc)
{

    auto yoloEngine = Yolo::create_infer("purple.trtmodel", Yolo::Type::V5, 0, 0.8f, 0.5f);

    while (1)
    {

        if (prefuture.valid())
        {
            pthread_mutex_lock(&infer_mutex);

            pthread_mutex_lock(&buff_mutex);

            auto bboxes = yoloEngine->commit(*matBuff).get();

            pthread_mutex_unlock(&buff_mutex);
            usleep(1000);

            int color_num[7] = {156, 43, 46, 180, 255, 255, 0x02};

            uint32_t NUM = bboxes.size();

            Detect->detect_distance_no_threead(bboxes, *matBuff2, NUM);
        }
        else
        {
            prefuture = yoloEngine->commit(*matBuff);
        }

        pthread_mutex_lock(&buff_mutex);

        cv::imshow("purple", *matBuff2);
        cv::waitKey(1);

        pthread_mutex_unlock(&buff_mutex);
        pthread_mutex_unlock(&infer_mutex);

        usleep(1000);
    }

    pthread_exit(NULL);
}

void *picture2(void *argc)
{
    auto yoloEngine = Yolo::create_infer("purple.trtmodel", Yolo::Type::V5, 0, 0.8f, 0.5f);

    while (1)
    {

        if (prefuture.valid())
        {
            pthread_mutex_lock(&protect_picture);
            pthread_mutex_lock(&buff_mutex);

            auto bboxes = yoloEngine->commit(*matBuff).get();

            pthread_mutex_unlock(&buff_mutex);
            
            usleep(100);

            int color_num[7] = {125, 43, 46, 155, 255, 255, 0x03};

            uint32_t NUM = bboxes.size();

            pthread_mutex_lock(&buff_mutex);

            // ObjectDetector::Box boxes[NUM];

            for (auto box : bboxes)
            {
                Detect->detect_boxes(bboxes, *matBuff, NUM, *depthBuff2, k4aTransformation, k4aCalibration, color_num);
                // cout << "Purple success" << endl;
            }

            pthread_mutex_unlock(&buff_mutex);
            usleep(100);
        }
        else
        {
            prefuture = yoloEngine->commit(*matBuff);
        }

        //pthread_mutex_lock(&buff_mutex);

        //cv::imshow("purple", *matBuff);
        //cv::waitKey(1);

        //pthread_mutex_unlock(&buff_mutex);
        pthread_mutex_unlock(&protect_picture);

        usleep(100);
    }

    pthread_exit(NULL);
}

void *infer_labels(void *argc)
{
    auto yoloEngine = Yolo::create_infer("test.trtmodel", Yolo::Type::V5, 0, 0.8f, 0.5f);
    cv::VideoCapture capture;
    capture.open("/dev/came1");

    while (1)
    {   pthread_mutex_lock(&buff_mutex);
    
        cv::Mat frame;
        capture.read(frame);
        
        pthread_mutex_unlock(&buff_mutex);
        
        if (prefuture.valid())
        {

            //pthread_mutex_lock(&protect_picture);
            pthread_mutex_lock(&buff_mutex);

            prefuture = yoloEngine->commit(frame);
            auto bboxes = prefuture.get();

            pthread_mutex_unlock(&buff_mutex);
            usleep(100);

            uint32_t NUM = bboxes.size();
           

            pthread_mutex_lock(&buff_mutex);

            for (auto box : bboxes)
            {
                Detect->detect_labels(bboxes, frame, NUM);
            }

            pthread_mutex_unlock(&buff_mutex);
            usleep(100);
        }
        else
        {
            prefuture = yoloEngine->commit(frame);
        }

        pthread_mutex_lock(&buff_mutex);

        cv::imshow("labels", frame);
        cv::waitKey(1);

        pthread_mutex_unlock(&buff_mutex);
        //pthread_mutex_unlock(&protect_picture);

        usleep(100);
    }
    pthread_exit(NULL);
}
