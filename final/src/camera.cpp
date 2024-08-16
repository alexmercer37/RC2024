/*
 * @Author: ddxy
 * @Date: 2023-10-10 10:43:39
 * @LastEditors: Tommy0929 tommy07210728@163.com
 * @FilePath: /final/src/camera.cpp
 * WHUROBOCON_SAVED!!!
 */ \
#include "../inc/camera.h"

#define PI 3.1415926535

using namespace std;
using namespace k4a;
using namespace cv;

void camera::init_kinect(k4a::device &device, k4a::capture &capture, k4a::transformation &k4aTransformation, k4a::calibration &k4aCalibration)
{

  device_count = device::get_installed_count();
  if (device_count == 0)
  {
    cout << "Error:no K4A devices found." << endl;
    return;
  }
  else
  {
    cout << "Found" << device_count << "connected devices." << endl;
  }

  device = device::open(K4A_DEVICE_DEFAULT);
  //device = device::open(0);
  cout << "Done:open device." << endl;
  init = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  init.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; // A是alpha，具有alpha纹理格式的颜色
  init.color_resolution = K4A_COLOR_RESOLUTION_720P; // 1920*1080
  init.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;    // 640*576
  init.camera_fps = K4A_FRAMES_PER_SECOND_30;        // 30帧
  init.synchronized_images_only = true;              // 只支持同步图像

  device.start_cameras(&init);
  cout << "Done:start camera." << endl;
  device.start_imu();
  cout << "Done:start imu." << endl;

  k4aCalibration = device.get_calibration(init.depth_mode, init.color_resolution);
  k4aTransformation = k4a::transformation(k4aCalibration);

  int iAuto = 0;
  while (1)
  {
    if (device.get_capture(&capture))
      cout << iAuto << ". Capture several frames to give auto-exposure" << endl;
    if (iAuto < 30)
    {
      iAuto++;
      continue;
    }
    else
    {
      cout << "Done: auto-exposure" << endl;
      break;
    }
  }
}

void camera::getAngel(k4a::device &device)
{
  __u8 buff = 0x01;
  __s16 angle[3]={0};

  k4a_imu_sample_t *imu_sample = new k4a_imu_sample_t;
  device.get_imu_sample(imu_sample, std::chrono::milliseconds(1000));

  // if (device.get_imu_sample(imu_sample, std::chrono::milliseconds(100)))
  // cout << "imu success" << endl;
  // else cout << "imu error" << endl;

  if (imu_sample != NULL)
  {
    // cout << imu_sample->temperature << endl;
    angle[0] = {-atan(imu_sample->acc_sample.xyz.x / pow(pow(imu_sample->acc_sample.xyz.z, 2) + pow(imu_sample->acc_sample.xyz.y, 2), 0.5)) * 180 / PI};
    cout<<angle[0]<<endl;
    sleep(0.01);
    libtty_write(fd, angle, buff);
  }
}

void camera::picture_update(k4a::device &device, k4a::capture &capture)
{
  device.get_capture(&capture, std::chrono::milliseconds(100));
}

cv::Mat *camera::getpicture(k4a::device &device, k4a::capture &capture, cv::Mat &cv_color, k4a::transformation &k4aTransformation)
{
 // device.get_capture(&capture, std::chrono::milliseconds(100));

  k4a_color = capture.get_color_image();

  cv::Mat *cv_color1;
  cv_color1 = new Mat(k4a_color.get_height_pixels(), k4a_color.get_width_pixels(), CV_8UC4, k4a_color.get_buffer());

  return cv_color1;

  // GaussianBlur(cv_color1, cv_color, cv::Size(5, 5), 3, 3);

  // bilateralFilter(cv_color1, cv_color, 9, 50, 5);
  // cvtColor(cv_color, cv_color, cv::COLOR_BGRA2BGR);

  // cv_infrared = Mat(k4a_infrared.get_height_pixels(), k4a_infrared.get_width_pixels(), CV_16U, k4a_infrared.get_buffer());

  // cv_infrared.convertTo(cv_infrared, CV_8U, 1);
}
cv::Mat camera::getpicture1(k4a::capture &capture, cv::Mat &cv_color, k4a::transformation &k4aTransformation)
{
  k4a_color = capture.get_color_image();

  cv::Mat cv_color1;
  cv_color1 = Mat(k4a_color.get_height_pixels(), k4a_color.get_width_pixels(), CV_8UC4, k4a_color.get_buffer());

  return cv_color1;

  // GaussianBlur(cv_color1, cv_color, cv::Size(5, 5), 3, 3);

  // bilateralFilter(cv_color1, cv_color, 9, 50, 5);
  // cvtColor(cv_color, cv_color, cv::COLOR_BGRA2BGR);

  // cv_infrared = Mat(k4a_infrared.get_height_pixels(), k4a_infrared.get_width_pixels(), CV_16U, k4a_infrared.get_buffer());

  // cv_infrared.convertTo(cv_infrared, CV_8U, 1);
}
cv::Mat *camera::getdepth(k4a::device &device, k4a::capture &capture, cv::Mat &cv_depth, k4a::transformation &k4aTransformation)
{
  //device.get_capture(&capture, std::chrono::milliseconds(100));

  k4a_depth = capture.get_depth_image();

  cv::Mat *cv_depth1;
  k4a_tf_depth = k4aTransformation.depth_image_to_color_camera(k4a_depth);
  cv_depth1 = new Mat(k4a_tf_depth.get_height_pixels(), k4a_tf_depth.get_width_pixels(), CV_16U, k4a_tf_depth.get_buffer());
  // (*cv_depth1).convertTo(cv_depth, CV_8U, 1);
 // usleep(100);

  return cv_depth1;
}

cv::Mat *camera::getir(k4a::device &device, k4a::capture &capture, cv::Mat &cv_infrared, k4a::transformation &k4aTransformation)
{

  k4a_infrared = capture.get_ir_image();

  cv::Mat *cv_infrared1;

  cv_infrared1 = new Mat(k4a_infrared.get_height_pixels(), k4a_infrared.get_width_pixels(), CV_16U, k4a_infrared.get_buffer());
  (*cv_infrared1).convertTo(cv_infrared, CV_8U, 1);
  return cv_infrared1;
}

void camera::picture_update(k4a::capture &capture, cv::Mat &cv_color, cv::Mat &cv_depth, k4a::transformation &k4aTransformation)
{
  if (device.get_capture(&capture, std::chrono::milliseconds(100)))
  {
    k4a_color = capture.get_color_image();
    k4a_depth = capture.get_depth_image();
    k4a_tf_depth = k4aTransformation.depth_image_to_color_camera(k4a_depth);
    cv_color = cv::Mat(k4a_color.get_height_pixels(), k4a_color.get_width_pixels(), CV_8UC4, (void *)k4a_color.get_buffer());
    cv::cvtColor(cv_color, cv_color, cv::COLOR_BGRA2BGR);
    cv_depth =
        cv::Mat(k4a_tf_depth.get_height_pixels(), k4a_tf_depth.get_width_pixels(), CV_16U, (void *)k4a_tf_depth.get_buffer(), static_cast<size_t>(k4a_tf_depth.get_stride_bytes()));
    // depthFrame.convertTo(depthFrame, CV_8U, 1);
  }
}

void camera::stopCamera()
{
  device.close();
}

void camera::getpicture4(k4a::capture &capture, cv::Mat &cv_color1, cv::Mat &cv_color, k4a::transformation &k4aTransformation)
{
  if (device.get_capture(&capture, std::chrono::milliseconds(100)))
  {
    k4a_color = capture.get_color_image();
    // k4a_depth = capture.get_depth_image();
    // k4a_infrared = capture.get_ir_image();

    // k4a_tf_depth = k4aTransformation.depth_image_to_color_camera(k4a_depth);

    cv_color1 = Mat(k4a_color.get_height_pixels(), k4a_color.get_width_pixels(), CV_8UC4, k4a_color.get_buffer());

    GaussianBlur(cv_color1, cv_color, cv::Size(5, 5), 3, 3);

    // bilateralFilter(cv_color1, cv_color, 9, 50, 5);

    cvtColor(cv_color, cv_color, cv::COLOR_BGRA2BGR);

    // cv_depth = Mat(k4a_tf_depth.get_height_pixels(), k4a_tf_depth.get_width_pixels(), CV_16U, k4a_tf_depth.get_buffer());
    // cv_infrared = Mat(k4a_infrared.get_height_pixels(), k4a_infrared.get_width_pixels(), CV_16U, k4a_infrared.get_buffer());
    // cv_depth.convertTo(cv_depth, CV_8U, 1);
    // cv_infrared.convertTo(cv_infrared, CV_8U, 1);
  }
}
