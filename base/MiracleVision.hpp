/**
 * @file MiracleVision.hpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief Main Function
 * @date 2024-02-20
 * @copyright Copyright (c) 2024 dgsyrc
 */
#pragma once

#include <chrono>
#include <ctime>

#include <fmt/color.h>
#include <fmt/core.h>

#include <opencv2/core.hpp>

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <onnxruntime/onnxruntime_c_api.h>

#include "devices/camera/mv_video_capture.hpp"
#include "devices/serial/uart_serial.hpp"
#include "module/armor/basic_armor.hpp"
#include "module/armor/DNN_armor.hpp"
#include "module/buff/basic_buff.hpp"
#include "module/ml/onnx_inferring.hpp"
#include "module/roi/basic_roi.hpp"
#include "utils/reset_mv_camera.hpp"
#include "module/camera/camera_calibration.hpp"
#include "module/angle_solve/angle_solve.hpp"
#include "module/buff/new_buff.hpp"
#include "utils/debug_tools.hpp"
#include "module/tracker/ppm.hpp"
#include "module/filter/basic_kalman.hpp"

#include <string>

auto idntifier = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "MiracleVision");

int cap_fps = 30;
cv::VideoWriter writer;
time_t time_now = time(0);
bool fire;
bool test_fps;

// Processed Frame Conuter
long long rec_time = 0;

int rec_cnt = 0;