/**
 * @file new_buff.hpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief buff
 * @date 2024-02-20
 * @copyright Copyright (c) 2024 dgsyrc
 *
 */

#pragma once

#define PI 3.14159265

#include <fmt/color.h>
#include <fmt/core.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

#include "utils/fps.hpp"
#include "module/filter/basic_kalman.hpp"

namespace new_buff
{
    /***
     * @brief 检测模式
     */
    cv::Mat ele_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)); // 椭圆内核
    enum Check_Moudle
    {
        ACTION_MODE,
        INACTION_MODE,
        CIRCLE_MODE,
    };

    cv::Point2f armor_last;
    cv::Point2f armor_now;

    cv::RotatedRect object_rects;

    cv::RotatedRect armor_now_rects;
    cv::RotatedRect armor_last_rects;

    basic_kalman::firstKalman kal;

    float tan_angle;

    bool isfindcircleR = false;
    bool isFindTarget = false;

    fps::FPS new_buff_fps;

    cv::Point2f circles;
    cv::Point2f last_circles;
    double area_circles = 0.0f;

    int brx;
    int bry;
    int tlx;
    int tly;

    double now_r = 0.0f;
    double last_r = 0.0f;

    struct armor_check
    {
        cv::Point2f cord[4];
        cv::Point2f obj;
        bool isPass = false;
        float d;
    };

    class buff
    {
    public:
        enum predict_status
        {
            MAX,
            MIN,
            NONE,
        };
        void getcontour(cv::Mat imgDil, cv::Mat img, cv::Mat &img_src, new_buff::Check_Moudle moudle);

        void set_config(std::string config_path);

        cv::Point2f stablePerdict(cv::Point2f &circle_r, cv::Mat &img);

        void predict(cv::Point2f &circle_r, cv::Mat &img);

        cv::Point2f calculateCord(cv::Point2f &circle_r);

        void main_buff_checker(cv::Mat img, cv::Mat img_src, new_buff::Check_Moudle moudle);

        cv::Point2f calculateCircleR(cv::Point2f P1, cv::Point2f P2, cv::Point2f P3);

        cv::Point2f returnCircleR();

        double returnDistance(cv::Point2f x, cv::Point2f y);

        cv::RotatedRect returnArmorRect();

        bool returnCenterDistance();

    private:
        struct info
        {
            float armor_height;
            float armor_lenght;
            float pic_armor_height;
            float pic_armor_lenght;
            float pic_distance;
            float armor_distance;
        } config;

        predict_status status = NONE;

        float last_velocity = 0.0;
        float now_velocity = 0.0;
        float min_velocity = 0.0;
        float max_velocity = 0.0;

        double timex;
        double last_timex;
        float T;

        bool start_T = false; // 检测到第一个最值标志

        cv::Point2f relay;
        bool first_T_finish = false;
        float arg[3] = {0};
        float alpha = 0.0;
        float omega = 0.0;
        float max_v = 0.0;
        float min_v = 0.0;
        float d_t;
        float velocity;
        float A = 1.0;

        float R;

        int sign = 1;

        float beta = 0.0;
        cv::Point2f object;

        double center_dist;
    };

}
