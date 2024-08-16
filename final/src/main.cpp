/*
 * @Author: ddxy
 * @Date: 2023-10-10 10:43:39
 * @LastEditors: Tommy0929 tommy07210728@163.com
 * @FilePath: /final/src/main.cpp
 * WHUROBOCON_SAVED!!!
 */ \
#include "../inc/main.h"

int main(int argc, char const *argv[])
{

      TRT::compile(
      TRT::Mode::FP16,
      1,
      "/home/nf/Downloads/3/final/workspace/blue.onnx",
      "blue.trtmodel");
    cout << "Done" << endl;

    //uart_init();

    camera_ptr = new camera;

    pthread_mutex_init(&protect_picture, nullptr);
    pthread_mutex_init(&buff_mutex, nullptr);
    pthread_mutex_init(&infer_mutex, nullptr);

    matBuff.reset(new cv::Mat);
    matBuff2.reset(new cv::Mat);
   // matBuff3.reset(new cv::Mat);
    depthBuff.reset(new cv::Mat);
    depthBuff2.reset(new cv::Mat);

    //camera_ptr->init_kinect(Device, capture, k4aTransformation, k4aCalibration);

    pthread_t threads[10] = {0};
    tcflush(fd, TCIOFLUSH);
    int receive_data;

   while (1)
   {
      uart_init();
      sleep(1);
      libtty_receive(fd, receive_data);

      cout << receive_data << endl;

      if (receive_data == 1)
      {  
         camera_ptr->init_kinect(Device, capture, k4aTransformation, k4aCalibration);
         pthread_create(&threads[0], NULL, k4aUpdate, (void *)camera_ptr);
         pthread_create(&threads[1], NULL, create_infer, NULL);
	 //pthread_create(&threads[3], NULL, infer_labels, NULL);

         receive_data = 0;

         while (1)
            usleep(100);
      }
      else if (receive_data == 2)
      {
         camera_ptr->init_kinect(Device, capture, k4aTransformation, k4aCalibration);
         pthread_create(&threads[0], NULL, k4aUpdate, (void *)camera_ptr);
         pthread_create(&threads[2], NULL, blue_infer, NULL);
	// pthread_create(&threads[3], NULL, infer_labels, NULL);

         receive_data = 0;
         while (1)
            usleep(100);
       }
       
   }

   // pthread_create(&threads[0], NULL, k4aUpdate, (void *)camera_ptr);
   // pthread_create(&threads[1], NULL, create_infer, NULL);
   // pthread_create(&threads[2], NULL, blue_infer, NULL);
   // pthread_create(&threads[2], NULL, picture2, NULL);
    //pthread_create(&threads[3], NULL, init_usb_camera, NULL);

   // pthread_create(&threads[3], NULL, infer_labels, NULL);
    // pthread_create(&threads[3], NULL, picture, NULL);

    //while (1)
       //usleep(100) ;

    delete (camera_ptr);

    return 0;
}
