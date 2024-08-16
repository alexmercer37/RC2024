/*
 * @Author: ddxy
 * @Date: 2023-10-10 10:43:39
 * @LastEditors: Tommy0929 tommy07210728@163.com
 * @FilePath: /final/src/detect.cpp
 * WHUROBOCON_SAVED!!!
 */ \
#include "../inc/detect.h"

using namespace std;

struct ObjectDetector::Box box;
pthread_mutex_t picture_mutex;
detect *det;
sem_t sems[15];

cv::Mat mask, output;

float detect::distance(float h)
{
    int foc = 645;
    double real_height = 7.48;
    float dis_inch = (real_height * foc) / (h);
    float dis_cm = dis_inch * 2.54;
    return dis_cm;
}

void detect::detect_distance_no_threead(ObjectDetector::BoxArray &bboxes, cv::Mat color, uint32_t &NUM)
{

    int imgHeight = color.rows;
    int imgWidth = color.cols;
    int left, top, right, bottom;
    int i = 0;

    for (auto &box : bboxes)
    {

        int left = (int)box.left;
        if (left < 0)
            left = 0;

        int top = (int)box.top;
        if (top < 0)
            top = 0;

        int right = (int)box.right;
        if (right > imgWidth)
            right = imgWidth;

        int bottom = (int)box.bottom;
        if (bottom > imgHeight)
            bottom = imgHeight;

        float h = bottom - top;
        float data[2] = {0};

        if ((right - left) > (bottom - top) && (right - left) < 1.2 * (bottom - top))
        {

            float dis = 0;

            dis = det->distance(h);

            cout << dis << endl;

            cv::Rect select = cv::Rect(left, top, right - left, bottom - top);

            uint8_t r, g, b;

            cv::rectangle(color, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(b, g, r), 3);

            cv::line(color, cv::Point(imgWidth * 0.5, top + (bottom - top) * 0.5), cv::Point(left + (right - left) * 0.5, top + (bottom - top) * 0.5), cv::Scalar(255, 0, 0));

            cout << (left + (right - left) * 0.5 - imgWidth * 0.5) * dis / 600 << endl;

            char Data[10];
            sprintf(Data, "%f", dis);

            cv::putText(color, Data, cv::Point(left + (right - left) * 0.5, top + (bottom - top) * 0.5 - 100), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255));

            data[0] = dis;
            data[1] = (left + (right - left) * 0.5 - imgWidth * 0.5) * dis / 600;

            // if (data[0] > 300 && data[1] > 300)
            // {
            //     libtty_write(fd, data);
            // }

            i++;
        }
    }
}

void detect::time()
{
    clock_t start, end;
    auto yoloStart = std::chrono::system_clock::now();
    auto yoloEnd = std::chrono::system_clock::now();
    auto yoloDuration = std::chrono::duration_cast<std::chrono::microseconds>(yoloEnd - yoloStart);
    std::cout << "yolo:" << double(yoloDuration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << "s" << std::endl;
}

void *detect_purple_pthread(void *args)
{

    args_ptr *ptr = static_cast<args_ptr *>(args);
    ObjectDetector::Box *box = ptr->box_ptr;

    int imgHeight = ptr->imgHeight;
    int imgWidth = ptr->imgWidth;
    int i = ptr->i;
    cv::Mat rgb = ptr->cv_rgb;

    int left = (int)box->left;
    if (left < 0)
        left = 0;

    int top = (int)box->top;
    if (top < 0)
        top = 0;

    int right = (int)box->right;
    if (right > imgWidth)
        right = imgWidth;

    int bottom = (int)box->bottom;
    if (bottom > imgHeight)
        bottom = imgHeight;

    float h = bottom - top;
    float data[2] = {0};

    sem_post(sems + i);

    if ((right - left) > (bottom - top) && (right - left) < 1.2 * (bottom - top))
    {

        float dis = 0;

        dis = det->distance(h);

        cout << dis << endl;

        cv::Rect select = cv::Rect(left, top, right - left, bottom - top);

        uint8_t r, g, b;

        cv::rectangle(rgb, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(b, g, r), 3);

        cv::line(rgb, cv::Point(imgWidth * 0.5, top + (bottom - top) * 0.5), cv::Point(left + (right - left) * 0.5, top + (bottom - top) * 0.5), cv::Scalar(255, 0, 0));

        cout << (left + (right - left) * 0.5 - imgWidth * 0.5) * dis / 600 << endl;

        char Data[6];
        sprintf(Data, "%f", dis);

        cv::putText(rgb, Data, cv::Point(left + (right - left) * 0.5, top + (bottom - top) * 0.5 - 100), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255));

        data[0] = dis;
        data[1] = (left + (right - left) * 0.5 - imgWidth * 0.5) * dis / 600;

        // if (data[0] > 300 && data[1] > 300)
        // {
        //     libtty_write(fd, data);
        // }
    }
    pthread_exit(NULL);
}

void detect::dstance_pthread(ObjectDetector::BoxArray &bboxes, cv::Mat rgb, uint32_t &NUM)
{
    int imgHeight = rgb.rows;
    int imgWidth = rgb.cols;
    int left, top, right, bottom;
    int i = 0;

    uint8_t r, g, b;

    ObjectDetector::Box boxes[NUM];
    for (auto &box : bboxes)
    {
        sem_init(sems + i, 0, 0);

        memcpy(boxes + i, &box, sizeof(box));

        args_ptr args;
        args.cv_rgb = rgb;
        args.box_ptr = boxes + i;
        args.imgHeight = imgHeight;
        args.imgWidth = imgWidth;
        args.i = i >= 15 ? 14 : i;
        pthread_t threads[NUM] = {0};
        pthread_create(&threads[i], NULL, detect_purple_pthread, (void *)&args);

        sem_wait(sems + i);
        sem_destroy(sems + i);

        i++;
    }
}

float *detect::distance(cv::Mat &rgb, int left, int right, int top, int bottom)
{
    float h = bottom - top;
    float *data = new float[2];
    if ((right - left) > 0.8 * (bottom - top) && (right - left) < 1.2 * (bottom - top))
    {

        float dis = 0;

        dis = det->distance(h);

        cout << dis << endl;

        cv::line(rgb, cv::Point(rgb.cols * 0.5, top + (bottom - top) * 0.5), cv::Point(left + (right - left) * 0.5, top + (bottom - top) * 0.5), cv::Scalar(255, 0, 0));

        cout << (left + (right - left) * 0.5 - rgb.cols * 0.5) * dis / 600 << endl;

        char Data[10];
        sprintf(Data, "%f", dis);

        cv::putText(rgb, Data, cv::Point(left + (right - left) * 0.5, top + (bottom - top) * 0.5 - 100), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255));

        data[0] = dis;
        data[1] = (left + (right - left) * 0.5 - rgb.cols * 0.5) * dis / 600;

        // if (data[0] > 300 && data[1] > 300)
        // {
        // libtty_write(fd, data, buff);
        // }
        return data;
    }
}

float getlength(int left, int right, int top, int bottom)
{
    float h = bottom - top;
    if ((right - left) > 0.8 * (bottom - top) && (right - left) < 1.2 * (bottom - top))
    {

        float length = 0;
        length = det->distance(h);

        return length;
    }
}

float getwidth(int left, int right, int top, int bottom)
{
    float h = bottom - top;
    if ((right - left) > 0.8 * (bottom - top) && (right - left) < 1.2 * (bottom - top))
    {

        float dis = getlength(left, right, top, bottom);
        float width = (left + (right - left) * 0.5 - 1280 * 0.5) * dis / 645;

        return width;
    }
}

float getheigth(int left, int right, int top, int bottom)
{
    float h = bottom - top;
    if ((right - left) > 0.8 * (bottom - top) && (right - left) < 1.2 * (bottom - top))
    {

        float dis = getlength(left, right, top, bottom);
        float heigth = (top + (bottom - top) * 0.5 - 720 * 0.5) * dis / 645;

        return heigth;
    }
}

void detect::detect_boxes(ObjectDetector::BoxArray &bboxes, cv::Mat rgb, uint32_t &NUM, cv::Mat &cv_depth, k4a::transformation &k4aTransformation, k4a::calibration &k4aCalibration, const int color_num[])
{
    int imgHeight = rgb.rows;
    int imgWidth = rgb.cols;

    int left, top, right, bottom;
    int i = 0;
    __u8 buff = 0x04;

    cv::Rect select;

    int square[NUM][4];
    int max[4] = {0};
    float dis_send[NUM][3];

    // std::vector<Pointcloud> l(NUM);
    std::vector<cameraCV> cameraCVs(NUM);
    std::vector<lcloud> lclouds(NUM);
    // std::vector<std::thread> threads(NUM);

    uint8_t r, g, b;

    for (int i = 0; i < NUM; i++)
    {
        auto box = bboxes[i];
        left = (int)box.left;
        if (left < 0)
            left = 0;

        square[i][0] = left;

        top = (int)box.top;
        if (top < 0)
            top = 0;

        square[i][1] = top;

        right = (int)box.right;
        if (right > imgWidth)
            right = imgWidth;

        square[i][2] = right;

        bottom = (int)box.bottom;
        if (bottom > imgHeight)
            bottom = imgHeight;

        square[i][3] = bottom;

        dis_send[i][0] = getlength(left, right, top, bottom);
        dis_send[i][1] = getwidth(left, right, top, bottom);
        dis_send[i][2] = getheigth(left, right, top, bottom);

        cv::rectangle(rgb, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(b, g, r), 3);

        for (int j = 0; j < 4; j++)
        {
            max[j] = square[0][j];
        }
    }

    for (int i = 0; i < NUM; i++)
    {
        // float x = dis_send[i][0]*(-0.75)-0.5*dis_send[i][1]+0.433*dis_send[i][2];
        // float y = dis_send[i][0]*(-0.433)+0.866*dis_send[i][1]+0.25*dis_send[i][2];
        cout << dis_send[i][0] << " " << dis_send[i][1] << " " << dis_send[i][2] << endl;
        // cout<<x<<" "<<y<<endl;
    }

    Libtty_Write(fd, dis_send, buff, NUM);

    for (int i = 0; i < NUM; i++)
    {
        if ((max[2] - max[0]) * (max[3] - max[1]) < ((square[i][2] - square[i][0]) * (square[i][3] - square[i][1])))
        {
            for (int j = 0; j < 4; j++)
                max[j] = square[i][j];
        }
    }

    select = cv::Rect(max[0], max[1], max[2] - max[0], max[3] - max[1]);

    // det->distance(rgb, max[0], max[2], max[1], max[3]);

    // cv::rectangle(rgb, cv::Point(max[0], max[1]), cv::Point(max[2], max[3]), cv::Scalar(b, g, r), 3);

    cameraCVs[i].getColor(rgb(select), mask, output, color_num);

    cv::Mat depthCut = cv::Mat::zeros(cv::Size(imgWidth, imgHeight), CV_16U);

    cv_depth(select).copyTo(depthCut(select), mask);

    lclouds[i].getXYZPointCloud(k4aTransformation, k4aCalibration, depthCut);

    lclouds[i].getPLY(color_num[6]);

    // threads[i] = std::thread(&lcloud::getPLY, &lclouds[i], std::ref(l[i]), rgb);

    lclouds[i].clearCloud();

    mask.release();
    lclouds.clear();
    cameraCVs.clear();

    // for (auto &entry : threads)
    //     entry.join();
    // threads.clear();
}

std::vector<float> detect::findMinSecondElement(const std::vector<std::vector<float>> &group)
{

    auto it = std::min_element(group.begin(), group.end(), [](const std::vector<float> &a, const std::vector<float> &b)
                               { return a[1] < b[1]; });

    if (it != group.end())
    {

        return *it;
    }
}

void detect::getMinData(std::vector<std::vector<float>> &values, float threshold)
{
    std::unordered_map<int, std::pair<int, float>> group_stats;
    std::unordered_map<int, std::vector<std::vector<float>>> groups;
    float result[15] = {0};

    int count = 0;
    int index = 0;
    int num = 0;
    __u8 BUFF = 0x01;

    for (const auto &value : values)
    {
        bool addedToGroup = false;
        int currentValue = value[0];

        for (auto &groupPair : groups)
        {
            int groupKey = groupPair.first;
            int diff = std::abs(currentValue - groupKey);

            if (diff <= threshold)
            {
                groupPair.second.push_back(value);
                group_stats[groupKey].first++;
                group_stats[groupKey].second += currentValue;

                addedToGroup = true;

                break;
            }
        }

        if (!addedToGroup)
        {
            groups[currentValue].push_back(value);
            group_stats[currentValue] = {1, currentValue};
        }
    }

    // for (const auto &groupPair : groups)
    // {
    //     std::cout << "Group with key " << groupPair.first << ":" << std::endl;
    //     for (const auto &vec : groupPair.second)
    //     {
    //         std::cout << "[" << vec[0] << ", " << vec[1] << "] " << std::endl;
    //     }
    //     std::cout << std::endl;
    // }

    // for (const auto &groupPair : groups)
    // {
    //     std::vector<float> minElement = findMinSecondElement(groupPair.second);

    //     if (!minElement.empty())
    //     {
    //         result.push_back(minElement);
    //         count += 1;
    //     }
    // }

    for (auto &groupPair : group_stats)
    {
        int count = groupPair.second.first;
        float sum = groupPair.second.second;
        float average = sum / count;

        result[index++] = average;
        result[index++] = count;
        num += 1;
    }
    // std::vector<float> data(2 * count + 1);
    // data[0] = count;

    // for (int i = 0; i < count; ++i)
    // {

    // cout << result[i][0] << " " << result[i][1] << endl;
    //     data[i * 2 + 1] = result[i][0];
    //     data[i * 2 + 2] = result[i][1];
    // }

    // for (int i = 0; i < 2 * count + 1; ++i)
    // {
    //     cout << data[i] << endl;
    // }
    float data[11] = {0};
    data[0] = num;

    for (int i = 0; i < 10; i++)
    {
        data[i + 1] = result[i];
    }

    for (int i = 0; i < 11; i++)
    {
        cout << data[i] << endl;
    }

    libtty_Write(fd, data, BUFF);
}

void detect::detect_labels(ObjectDetector::BoxArray &bboxes, cv::Mat rgb, uint32_t &NUM)
{
    int imgHeight = rgb.rows;
    int imgWidth = rgb.cols;
    int left, top, right, bottom;
    float h, dis, left_dis, top_dis;

    cv::Rect select;

    float data[2] = {0};

    std::vector<std::vector<float>> values(NUM, std::vector<float>(2));
    uint8_t r, g, b;

    for (int i = 0; i < bboxes.size(); i++)
    {
        auto box = bboxes[i];

        left = (int)box.left;
        if (left < 0)
            left = 0;

        top = (int)box.top;
        if (top < 0)
            top = 0;

        right = (int)box.right;
        if (right > imgWidth)
            right = imgWidth;

        bottom = (int)box.bottom;
        if (bottom > imgHeight)
            bottom = imgHeight;

        cv::rectangle(rgb, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(b, g, r), 3);

        h = bottom - top;
        dis = det->distance(h);

        left_dis = (-left - (right - left) * 0.5 + imgWidth * 0.5) * dis / 645;
        values[i][0] = left_dis;

        top_dis = (imgHeight * 0.5 - top - (bottom - top) * 0.5) * dis / 645;
        values[i][1] = top_dis;
    }

    float threshold = (right - left) * dis / 300;

    det->getMinData(values, threshold);
}
