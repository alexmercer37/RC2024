/*
 * @Author: ddxy
 * @Date: 2023-10-10 10:43:39
 * @LastEditors: Tommy0929 tommy07210728@163.com
 * @FilePath: /final/src/pointcloud.cpp
 * WHUROBOCON_SAVED!!!
 */
#include "../inc/pointcloud.h"

using namespace std;
using namespace pcl;
using namespace cv;

int fd;

lcloud::lcloud()
{
  source = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  source_downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  sphere = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloudT = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();

  inliers_sphere = std::make_shared<pcl::PointIndices>();
  coefficients_sphere = std::make_shared<pcl::ModelCoefficients>();

  count = 0;
}

void lcloud::getMaskAccordingToColor(const Mat &cv_rgbimg, Mat &mask)
{
  Mat hsvImage, mask1, mask2;

  cvtColor(cv_rgbimg, hsvImage, COLOR_BGR2HSV);

  inRange(hsvImage, Scalar(146, 43, 46), Scalar(180, 255, 255), mask);
}

void lcloud::getXYZPointCloud(const k4a::transformation &k4aTransformation, const k4a::calibration &k4aCalibration, const Mat &cv_depthimg)
{
  clock_t start, end;
  // start = clock();

  k4a::image depthImage{nullptr};
  k4a::image xyzImage{nullptr};
  k4a::image pointCloud{nullptr};

  int width = k4aCalibration.color_camera_calibration.resolution_width;
  int height = k4aCalibration.color_camera_calibration.resolution_height;

 // int width = cv_depthimg.cols;
 // int height = cv_depthimg.rows;

  depthImage = k4a::image::create_from_buffer(K4A_IMAGE_FORMAT_DEPTH16, width, height, width * (int)sizeof(uint16_t), cv_depthimg.data, width * height * 2, nullptr, nullptr);
  xyzImage = k4aTransformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_COLOR);

  auto *xyzImageData = (int16_t *)(void *)xyzImage.get_buffer();

  for (int i = 0; i < width * height; i+=4)
  {
    if (xyzImageData[3 * i + 2] == 0 || xyzImageData[3 * i + 2] >= 4000) // 要4m内

      continue;
    if (i % 3 != 0)

      continue;

    PointXYZ point;

    point.x = xyzImageData[3 * i + 0];
    point.y = xyzImageData[3 * i + 1];
    point.z = xyzImageData[3 * i + 2];

    source->points.push_back(point);

    // end = clock();
    // cout << "转点云：" << (double)(end - start) / CLOCKS_PER_SEC << endl;
  }

  pointCloud.reset();
  xyzImage.reset();
  depthImage.reset();
}

void lcloud::getPLY(Pointcloud &pointcloud)
{
  Eigen::Vector4f max_pt;
  Eigen::Vector4f min_pt;
  Eigen::Vector4f centroid;

  Eigen::Vector4f leaf_size{15, 15, 15, 0};

  clock_t start, end;

  search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());

  ConditionAnd<PointXYZ>::Ptr range_cond(new ConditionAnd<PointXYZ>()); // 所有条件都满足（or为满足一个即可）

  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 400.0)));
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -100.0)));

  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 2500.0)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 100.0)));

  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 100.0)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -500.0)));

  start = clock();

  filt.setCondition(range_cond);
  filt.setKeepOrganized(false);
  filt.setInputCloud(source);
  filt.filter(*source_downsampled);

  end = clock();

  // cout << "过滤：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

  // if (source_downsampled->size() < 100)
  // {
  //   cout << "点云数目太少" << endl;
  //   return;
  // }

  start = clock();

  vg.setInputCloud(source_downsampled);
  vg.setLeafSize(leaf_size);
  vg.setMinimumPointsNumberPerVoxel(1);
  vg.filter(*source_downsampled);

  end = clock();
  cout << "下采样：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

  getMinMax3D(*source_downsampled, min_pt, max_pt);

  // if (source_downsampled->size() < 70 || max_pt(1) < 40)
  // {
  // cout << "下采样点云数目太少" << endl;
  //   return;
  // }

  start = clock();

  sor.setInputCloud(source_downsampled);
  sor.setMeanK(50);
  sor.setStddevMulThresh(3);
  sor.filter(*source_downsampled);

  end = clock();
  // cout << "去除离群点：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

  // if (source_downsampled->size() < 30)
  // {
  //   cout << "去离群和直通滤波后点云数目太少" << endl;
  //   return;
  // }

  start = clock();

  ne.setInputCloud(source_downsampled);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(30.0);
  ne.compute(*cloud_normals);

  end = clock();
  // cout << "法线估计：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

  start = clock();

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(1);
  seg.setMaxIterations(10000);

  int i = 0;

  int nr_points = source_downsampled->points.size();

  float data[3] = {0};

  while (source_downsampled->points.size() > 0.01 * nr_points)
  {
    start = clock();

    seg.setInputCloud(source_downsampled);
   // seg.setInputNormals(cloud_normals);
    seg.segment(*inliers_sphere, *coefficients_sphere);

    if (inliers_sphere->indices.size() == 0)
    {
      // cout << "could not remove " << endl;
      break;
    }

    extract.setInputCloud(source_downsampled);
    extract.setIndices(inliers_sphere);
    extract.setNegative(false);
    extract.filter(*sphere);
    extract.setNegative(true);
    extract.filter(*cloudT);

    float x_temp = coefficients_sphere->values[0];
    float y_temp = coefficients_sphere->values[1];
    float z_temp = coefficients_sphere->values[2];

    data[0] = -z_temp * 0.866025 * 0.866025 - x_temp * 0.5 * 0.866025 + y_temp * 0.5 - 216.94;
    data[1] = x_temp * 0.866025 - z_temp * 0.5 + 259.1;
    data[2] = 442.78 - 95;

    pointcloud.x = data[0];
    pointcloud.y = data[1];
    pointcloud.z = data[2];

    // if (data[0] > 300 && data[1] > 300)
    // {
    //   libtty_write(fd, data);
    // }

    cout << "保存成功" << endl;

    i++;

    *source_downsampled = *cloudT;

    end = clock();
    // cout << "分割球体：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

    // count++;
  }
}

void lcloud::getPLY(__u8 buff)
{
  Eigen::Vector4f max_pt;
  Eigen::Vector4f min_pt;
  Eigen::Vector4f centroid;

  Eigen::Vector4f leaf_size{22, 22, 22, 0};

  clock_t start, end;

  search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());

  ConditionAnd<PointXYZ>::Ptr range_cond(new ConditionAnd<PointXYZ>()); // 所有条件都满足（or为满足一个即可）

  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 300.0)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -600.0)));

 range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 2000.0)));
 range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 300.0)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 400.0)));
 range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -400.0)));

 // start = clock();

 filt.setCondition(range_cond);
 filt.setKeepOrganized(false);
 filt.setInputCloud(source);
 filt.filter(*source_downsampled);


 // end = clock();

  // cout << "过滤：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

  // if (source_downsampled->size() < 100)
  // {
  //   cout << "点云数目太少" << endl;
  //   return;
  // }
  //
  //start = clock();

  vg.setInputCloud(source_downsampled);
  vg.setLeafSize(leaf_size);
  vg.setMinimumPointsNumberPerVoxel(1);
  vg.filter(*source_downsampled);

 // end = clock();
  // cout << "下采样：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

  getMinMax3D(*source_downsampled, min_pt, max_pt);

  // if (source_downsampled->size() < 70 || max_pt(1) < 40)
  // {
  // cout << "下采样点云数目太少" << endl;
  //   return;
  // }

 // start = clock();

  sor.setInputCloud(source_downsampled);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1);
  sor.filter(*source_downsampled);

 // end = clock();
  // cout << "去除离群点：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

  // if (source_downsampled->size() < 30)
  // {
  //   cout << "去离群和直通滤波后点云数目太少" << endl;
  //   return;
  // }

 // start = clock();

 // ne.setInputCloud(source_downsampled);
 // ne.setSearchMethod(tree);
 // ne.setRadiusSearch(30.0);
 // ne.compute(*cloud_normals);

 // end = clock();
  // cout << "法线估计：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

 // start = clock();

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(5);
  seg.setMaxIterations(10000);
  //seg.setOptimizeCoefficients(true);

  int i = 0;

  int nr_points = source_downsampled->points.size();

  __s16 data[3] = {0};

  while (source_downsampled->points.size() > 0.01 * nr_points)
  {
   // start = clock();

    seg.setInputCloud(source_downsampled);
   // seg.setInputNormals(cloud_normals);
    seg.segment(*inliers_sphere, *coefficients_sphere);

    if (inliers_sphere->indices.size() == 0)
    {
      // cout << "could not remove " << endl;
      break;
    }

    extract.setInputCloud(source_downsampled);
    extract.setIndices(inliers_sphere);
    extract.setNegative(false);
    extract.filter(*sphere);
    //extract.setNegative(true);
    //extract.filter(*cloudT);

    float x_temp = coefficients_sphere->values[0];
    float y_temp = coefficients_sphere->values[1];
    float z_temp = coefficients_sphere->values[2];

   // data[0] = -z_temp * 0.866025 * 0.866025 - x_temp * 0.5 * 0.866025 + y_temp * 0.5 - 194.82;
   // data[1] = x_temp * 0.866025 - z_temp * 0.5 + 235.18;
   // data[2] = 442.78 - 95;
     data[0] = x_temp;
     data[1] = y_temp;
     data[2] = z_temp;

     //cout << coefficients_sphere->values[0] << " " << coefficients_sphere->values[2] << " " << data[2] << endl;
    cout << data[0] << " " << data[1] << " " << data[2] << endl;

    if (data[0] < 600 && data[0] > (-600) && data[1] < 500 && data[1] > (-200) && data[2] > 200 &&data[2] < 2000)
    {
        libtty_write(fd, data, buff);
    }

    i++;

    *source_downsampled = *cloudT;

    end = clock();
     //cout << "分割球体：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

     count++;
  }
}

// void lcloud::getPLY()
// {
//   Eigen::Vector4f max_pt;
//   Eigen::Vector4f min_pt;
//   Eigen::Vector4f centroid;
//   clock_t start, end;
//   pcl::PLYWriter writer;
//   ConditionAnd<PointXYZ>::Ptr range_cond(new ConditionAnd<PointXYZ>()); // 所有条件都满足（or为满足一个即可）
//   range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 200.0)));
// range_cond->addxxxxxComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -100.0)));
//   range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 400.0)));
// range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 200.0)));
//   range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 200.0)));
// range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -50.0)));
//   start = clock();
//   filt.setCondition(range_cond);
//   filt.setKeepOrganized(false);
//   filt.setInputCloud(source);
//   filt.filter(*source_downsampled);
//   end = clock();
// cout << "过滤：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

// if (source_downsampled->size() < 100)
// {
// cout << "点云数目太少" << endl;
//   return;
// }

//   start = clock();
//   vg.setInputCloud(source_downsampled);
// vg.setLeafSize(0.01f, 0.01f, 0.01f);
//   vg.filter(*source_downsampled);
//   end = clock();
//   cout << "下采样：" << (double)(end - start) / CLOCKS_PER_SEC << endl;
// io::savePLYFile("/home/ddxy/Downloads/kinect4/testcloud/source.ply", *source_downsampled);
// getMinMax3D(*source_downsampled, min_pt, max_pt);
// if (source_downsampled->size() < 70 || max_pt(1) < 40)
// {
// cout << "下采样点云数目太少" << endl;
//   return;
// }

//   start = clock();
//   sor.setInputCloud(source_downsampled);
//   sor.setMeanK(50);
//   sor.setStddevMulThresh(3);
//   sor.filter(*source_downsampled);
//   end = clock();
// cout << "去除离群点：" << (double)(end - start) / CLOCKS_PER_SEC << endl;
// if (source_downsampled->size() < 30)
// {
// cout << "去离群和直通滤波后点云数目太少" << endl;
//   return;
// }

//   start = clock();
//   search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
//   ne.setInputCloud(source_downsampled);
//   ne.setSearchMethod(tree);
//   ne.setRadiusSearch(30.0);
//   ne.compute(*cloud_normals);
//   end = clock();

// cout << "法线估计：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

// start = clock();
//   seg.setOptimizeCoefficients(true);
//   seg.setModelType(pcl::SACMODEL_SPHERE);
//   seg.setMethodType(pcl::SAC_RANSAC);
//   seg.setDistanceThreshold(1);
//   seg.setMaxIterations(10000);
//   seg.setInputCloud(source_downsampled);
//   seg.setInputNormals(cloud_normals);
//   seg.segment(*inliers_sphere, *coefficients_sphere);
//   extract.setInputCloud(source_downsampled);
//   extract.setIndices(inliers_sphere);
//   extract.setNegative(false);
//   extract.filter(*sphere);
//   end = clock();
//   cout << "过滤球体：" << (double)(end - start) / CLOCKS_PER_SEC << endl;
// float data[3];

// if (sphere->size() < 20)
// {
//   cout << "球体点云数量太少" << endl;
//   return;
// }
// if (count == 18)
// {
// pcl::io::savePLYFile("/home/ddxy/Downloads/视觉部分/kinect/camera/testcloud/source_downsampled.ply", *source_downsampled);
// pcl::io::savePLYFile("/home/ddxy/Downloads/视觉部分/kinect/camera/testcloud/sphere22.ply", *sphere);
//     cout << coefficients_sphere->values[0] << " " << coefficients_sphere->values[1] << " " << coefficients_sphere->values[2] << endl;
//     cout << "保存成功" << endl;

//     *data=coefficients_sphere->values[0] - 290;
//     *(data+1)=442.78 - 95;
//     *(data+2)=coefficients_sphere->values[2] + 224.111;
//   libtty_write(fd,data);
// }

void lcloud::clearCloud()
{
  source->clear();
  source_downsampled->clear();

  cloud_normals->clear();
  sphere->clear();
  cloudT->clear();

  inliers_sphere->header.frame_id.clear();
  inliers_sphere->indices.clear();

  coefficients_sphere->header.frame_id.clear();
  coefficients_sphere->values.clear();
}
