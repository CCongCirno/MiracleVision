/**
 * @file angle_solve.hpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief calculate angle pitch and yaw
 * @date 2024-02-20
 * @copyright Copyright (c) 2024 dgsyrc
 *
 */

#pragma once

#include <cstring>
#include <cmath>

#include <fmt/core.h>
#include <fmt/color.h>

#include <devices/serial/uart_serial.hpp>

#include <opencv2/opencv.hpp>

#include "module/tracker/ppm.hpp"

#define PI 3.14159265

auto angle_info = fmt::format(fg(fmt::color::pink) | fmt::emphasis::bold, "angle_solve_info");

namespace angle_solve
{
    class solve
    {
    public:
        void set_config(std::string config_path);

        void angleSolve(cv::RotatedRect object, int row, int col, uart::SerialPort &info);

        void angleSolvePPM(cv::RotatedRect object, int row, int col, Tracker::PPM &ppm_x, Tracker::PPM &ppm_y, uart::SerialPort &info);
        //

        float returnYawAngle();

        float returnPitchAngle();

        cv::Point getPPMTarget();

    private:
        int pos_last[2], pos[2], pos_current[2];
        int pos_infer[2];
        struct info
        {
            float armor_height;
            float armor_lenght;
            float pic_armor_height;
            float pic_armor_lenght;
            float pic_distance;
            float armor_distance;
            float speed_arg;
        } config;

        struct cord
        {
            cv::Point3d predict;
            float yaw;
            float pitch;
            float time;
        } target;

        struct gyro
        {
            float yaw;
            float pitch;
        } compensation;
    };

};