/**
 * @file basic_armor.cpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief armor identification
 * @date 2024-02-20
 * @copyright Copyright (c) 2024 dgsyrc
 *
 */
#include "basic_armor.hpp"
namespace basic_armor
{

  Detector::Detector(const std::string _armor_config)
  {
    cv::FileStorage fs_armor(_armor_config, cv::FileStorage::READ);
    // 初始化基本参数
    fs_armor["DEBUG_MODE"] >> armor_config_.debug_mode;
    fs_armor["WINDOW_SCALE"] >> armor_config_.window_scale;
    fs_armor["GRAY_EDIT"] >> image_config_.gray_edit;
    fs_armor["COLOR_EDIT"] >> image_config_.color_edit;
    fs_armor["METHOD"] >> image_config_.method;
    fs_armor["BLUE_ARMOR_GRAY_TH"] >> image_config_.blue_armor_gray_th;
    fs_armor["RED_ARMOR_GRAY_TH"] >> image_config_.red_armor_gray_th;

    if (image_config_.method == 0)
    {
      fs_armor["RED_ARMOR_COLOR_TH"] >> image_config_.red_armor_color_th;
      fs_armor["BLUE_ARMOR_COLOR_TH"] >> image_config_.blue_armor_color_th;
      fs_armor["GREEN_ARMOR_COLOR_TH"] >> image_config_.green_armor_color_th;
      fs_armor["WHITE_ARMOR_COLOR_TH"] >> image_config_.white_armor_color_th;
    }
    else
    {
      fs_armor["H_RED_MIN"] >> image_config_.h_red_min;
      fs_armor["H_RED_MAX"] >> image_config_.h_red_max;
      fs_armor["S_RED_MIN"] >> image_config_.s_red_min;
      fs_armor["S_RED_MAX"] >> image_config_.s_red_max;
      fs_armor["V_RED_MIN"] >> image_config_.v_red_min;
      fs_armor["V_RED_MAX"] >> image_config_.v_red_max;

      fs_armor["H_BLUE_MIN"] >> image_config_.h_blue_min;
      fs_armor["H_BLUE_MAX"] >> image_config_.h_blue_max;
      fs_armor["S_BLUE_MIN"] >> image_config_.s_blue_min;
      fs_armor["S_BLUE_MAX"] >> image_config_.s_blue_max;
      fs_armor["V_BLUE_MIN"] >> image_config_.v_blue_min;
      fs_armor["V_BLUE_MAX"] >> image_config_.v_blue_max;
    }

    fs_armor["B_RED_MIN"] >> image_config_.b_red_th_min;
    fs_armor["B_RED_MAX"] >> image_config_.b_red_th_max;
    fs_armor["G_RED_MIN"] >> image_config_.g_red_th_min;
    fs_armor["G_RED_MAX"] >> image_config_.g_red_th_max;
    fs_armor["R_RED_MIN"] >> image_config_.r_red_th_min;
    fs_armor["R_RED_MAX"] >> image_config_.r_red_th_max;

    fs_armor["B_BLUE_MIN"] >> image_config_.b_blue_th_min;
    fs_armor["B_BLUE_MAX"] >> image_config_.b_blue_th_max;
    fs_armor["G_BLUE_MIN"] >> image_config_.g_blue_th_min;
    fs_armor["G_BLUE_MAX"] >> image_config_.g_blue_th_max;
    fs_armor["R_BLUE_MIN"] >> image_config_.r_blue_th_min;
    fs_armor["R_BLUE_MAX"] >> image_config_.r_blue_th_max;

    fs_armor["LIGHT_DRAW"] >> light_config_.light_draw;
    fs_armor["LIGHT_EDTI"] >> light_config_.light_edit;

    fs_armor["LIGHT_RATIO_W_H_MIN"] >> light_config_.ratio_w_h_min;
    fs_armor["LIGHT_RATIO_W_H_MAX"] >> light_config_.ratio_w_h_max;

    fs_armor["LIGHT_ANGLE_MIN"] >> light_config_.angle_min;
    fs_armor["LIGHT_ANGLE_MAX"] >> light_config_.angle_max;

    fs_armor["LIGHT_PERIMETER_MIN"] >> light_config_.perimeter_min;
    fs_armor["LIGHT_PERIMETER_MAX"] >> light_config_.perimeter_max;

    fs_armor["ARMOR_EDIT"] >> armor_config_.armor_edit;
    fs_armor["ARMOR_DRAW"] >> armor_config_.armor_draw;
    fs_armor["ARMOR_FORECAST"] >> armor_config_.armor_forecast;
    fs_armor["ARMOR_HEIGHT_RATIO_MIN"] >> armor_config_.light_height_ratio_min;
    fs_armor["ARMOR_HEIGHT_RATIO_MAX"] >> armor_config_.light_height_ratio_max;

    fs_armor["ARMOR_WIDTH_RATIO_MIN"] >> armor_config_.light_width_ratio_min;
    fs_armor["ARMOR_WIDTH_RATIO_MAX"] >> armor_config_.light_width_ratio_max;

    fs_armor["ARMOR_Y_DIFFERENT"] >> armor_config_.light_y_different;
    fs_armor["ARMOR_HEIGHT_DIFFERENT"] >> armor_config_.light_height_different;
    fs_armor["ARMOR_ANGLE_DIFFERENT"] >> armor_config_.armor_angle_different;

    fs_armor["ARMOR_SMALL_ASPECT_MIN"] >> armor_config_.small_armor_aspect_min;
    fs_armor["ARMOR_TYPE_TH"] >> armor_config_.armor_type_th;
    fs_armor["ARMOR_BIG_ASPECT_MAX"] >> armor_config_.big_armor_aspect_max;
    fs_armor.release();

    fmt::print("[{}] Info, Armor configuration initial success\n", idntifier_green);
  }

  float Detector::getDistance(const cv::Point a, const cv::Point b)
  {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
  }

  void Detector::freeMemory(const std::string _armor_config)
  {
    cv::FileStorage fs_armor(_armor_config, cv::FileStorage::WRITE);
    lost_armor_success = armor_success;
    armor_success = false;

    light_.clear();
    light_.shrink_to_fit();
    armor_.clear();
    armor_.shrink_to_fit();

    fs_armor << "DEBUG_MODE" << armor_config_.debug_mode;
    fs_armor << "WINDOW_SCALE" << armor_config_.window_scale;
    fs_armor << "GRAY_EDIT" << image_config_.gray_edit;
    fs_armor << "COLOR_EDIT" << image_config_.color_edit;
    fs_armor << "METHOD" << image_config_.method;
    fs_armor << "BLUE_ARMOR_GRAY_TH" << image_config_.blue_armor_gray_th;
    fs_armor << "RED_ARMOR_GRAY_TH" << image_config_.red_armor_gray_th;

    fs_armor << "RED_ARMOR_COLOR_TH" << image_config_.red_armor_color_th;
    fs_armor << "BLUE_ARMOR_COLOR_TH" << image_config_.blue_armor_color_th;
    fs_armor << "GREEN_ARMOR_COLOR_TH" << image_config_.green_armor_color_th;
    fs_armor << "WHITE_ARMOR_COLOR_TH" << image_config_.white_armor_color_th;

    fs_armor << "H_RED_MIN" << image_config_.h_red_min;
    fs_armor << "H_RED_MAX" << image_config_.h_red_max;
    fs_armor << "S_RED_MIN" << image_config_.s_red_min;
    fs_armor << "S_RED_MAX" << image_config_.s_red_max;
    fs_armor << "V_RED_MIN" << image_config_.v_red_min;
    fs_armor << "V_RED_MAX" << image_config_.v_red_max;

    fs_armor << "H_BLUE_MIN" << image_config_.h_blue_min;
    fs_armor << "H_BLUE_MAX" << image_config_.h_blue_max;
    fs_armor << "S_BLUE_MIN" << image_config_.s_blue_min;
    fs_armor << "S_BLUE_MAX" << image_config_.s_blue_max;
    fs_armor << "V_BLUE_MIN" << image_config_.v_blue_min;
    fs_armor << "V_BLUE_MAX" << image_config_.v_blue_max;

    fs_armor << "B_RED_MIN" << image_config_.b_red_th_min;
    fs_armor << "B_RED_MAX" << image_config_.b_red_th_max;
    fs_armor << "G_RED_MIN" << image_config_.g_red_th_min;
    fs_armor << "G_RED_MAX" << image_config_.g_red_th_max;
    fs_armor << "R_RED_MIN" << image_config_.r_red_th_min;
    fs_armor << "R_RED_MAX" << image_config_.r_red_th_max;

    fs_armor << "B_BLUE_MIN" << image_config_.b_blue_th_min;
    fs_armor << "B_BLUE_MAX" << image_config_.b_blue_th_max;
    fs_armor << "G_BLUE_MIN" << image_config_.g_blue_th_min;
    fs_armor << "G_BLUE_MAX" << image_config_.g_blue_th_max;
    fs_armor << "R_BLUE_MIN" << image_config_.r_blue_th_min;
    fs_armor << "R_BLUE_MAX" << image_config_.r_blue_th_max;

    fs_armor << "LIGHT_EDTI" << light_config_.light_edit;
    fs_armor << "LIGHT_DRAW" << light_config_.light_draw;

    fs_armor << "LIGHT_RATIO_W_H_MIN" << light_config_.ratio_w_h_min;
    fs_armor << "LIGHT_RATIO_W_H_MAX" << light_config_.ratio_w_h_max;

    fs_armor << "LIGHT_ANGLE_MIN" << light_config_.angle_min;
    fs_armor << "LIGHT_ANGLE_MAX" << light_config_.angle_max;

    fs_armor << "LIGHT_PERIMETER_MIN" << light_config_.perimeter_min;
    fs_armor << "LIGHT_PERIMETER_MAX" << light_config_.perimeter_max;

    fs_armor << "ARMOR_FORECAST" << armor_config_.armor_forecast;
    fs_armor << "ARMOR_EDIT" << armor_config_.armor_edit;
    fs_armor << "ARMOR_DRAW" << armor_config_.armor_draw;
    fs_armor << "ARMOR_HEIGHT_RATIO_MIN" << armor_config_.light_height_ratio_min;
    fs_armor << "ARMOR_HEIGHT_RATIO_MAX" << armor_config_.light_height_ratio_max;

    fs_armor << "ARMOR_WIDTH_RATIO_MIN" << armor_config_.light_width_ratio_min;
    fs_armor << "ARMOR_WIDTH_RATIO_MAX" << armor_config_.light_width_ratio_max;

    fs_armor << "ARMOR_Y_DIFFERENT" << armor_config_.light_y_different;
    fs_armor << "ARMOR_HEIGHT_DIFFERENT" << armor_config_.light_height_different;
    fs_armor << "ARMOR_ANGLE_DIFFERENT" << armor_config_.armor_angle_different;

    fs_armor << "ARMOR_SMALL_ASPECT_MIN" << armor_config_.small_armor_aspect_min;
    fs_armor << "ARMOR_TYPE_TH" << armor_config_.armor_type_th;
    fs_armor << "ARMOR_BIG_ASPECT_MAX" << armor_config_.big_armor_aspect_max;

    fs_armor.release();
  }

  bool Detector::findLight(cv::Mat &src_img, int my_color)
  {
    int perimeter = 0;
    cv::RotatedRect box;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_color_img,
                     contours,
                     cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_NONE);
    // tools::Tools::imWindow("bin", bin_color_img, armor_config_.window_scale);
    //  std::cout << "c_size " << contours.size() << '\n';
    if (contours.size() < 2)
    {
      fmt::print("[{}] Info, quantity of contours less than 2\n", idntifier_green);
      return false;
    }
    // 调参开关
    if (light_config_.light_edit == 1 && !light_config_.light_edit)
    {
      std::string window_name = {"[basic_armor] findLight() -> light_trackbar"};
      light_config_.light_edit = true;
      cv::namedWindow(window_name);

      cv::createTrackbar("angle_min", window_name,
                         &light_config_.angle_min, 1800, NULL);
      cv::createTrackbar("angle_max", window_name,
                         &light_config_.angle_max, 1800, NULL);

      cv::createTrackbar("perimeter_min", window_name,
                         &light_config_.perimeter_min, 100000, NULL);
      cv::createTrackbar("perimeter_max", window_name,
                         &light_config_.perimeter_max, 100000, NULL);

      cv::createTrackbar("ratio_w_h_min", window_name,
                         &light_config_.ratio_w_h_min, 1000, NULL);
      cv::createTrackbar("ratio_w_h_max", window_name,
                         &light_config_.ratio_w_h_max, 1000, NULL);
      cv::imshow(window_name, light_trackbar_);
    }

    for (size_t i = 0; i != contours.size(); ++i)
    {
      perimeter = arcLength(contours[i], true);

      if (perimeter < light_config_.perimeter_min ||
          perimeter > light_config_.perimeter_max ||
          contours[i].size() < 5)
      {
        continue;
      }

      box = cv::fitEllipse(cv::Mat(contours[i]));
      if (!colorCheck(box, src_img, uart::Color(my_color)))
      {
        continue;
      }
      if (box.angle > 90.0f)
      {
        box.angle = box.angle - 180.0f;
      }

      static float _h = MAX(box.size.width, box.size.height);
      static float _w = MIN(box.size.width, box.size.height);
      static float light_w_h = _h / _w;
      // 判断灯条的条件
      if (box.angle < light_config_.angle_max &&
          box.angle > light_config_.angle_min &&
          light_w_h < light_config_.ratio_w_h_max / 10.0 &&
          light_w_h > light_config_.ratio_w_h_min / 10.0 &&
          box.size.height * box.size.width < 30000)
      {
        light_.emplace_back(box);
        if (light_config_.light_draw == 1 || light_config_.light_edit == 1)
        {
          cv::Point2f vertex[4];
          box.points(vertex);
          cv::putText(draw_img_, std::to_string(int(box.angle)), {int(vertex[1].x), int(vertex[1].y - 5)}, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
          for (size_t l = 0; l != 4; ++l)
          {
            cv::line(draw_img_, vertex[l], vertex[(l + 1) % 4], cv::Scalar(0, 255, 255), 3, 8);
          }
        }
      }
    }

    if (light_.size() < 2)
    {
      // fmt::print("[{}] Info, quantity of light bar less than two\n", idntifier_green);

      return false;
    }

    return true;
  }

  bool Detector::colorCheck(cv::RotatedRect &rect, cv::Mat &src_img, uart::Color my_color)
  {

    cv::Rect rect_ = rect.boundingRect();
    if (rect_.tl().x > src_img.cols || rect_.br().x > src_img.cols || rect_.tl().y > src_img.rows || rect_.br().y > src_img.rows || rect_.tl().x <= 0 || rect_.br().x <= 0 || rect_.tl().y <= 0 || rect_.br().y <= 0)
    {
      return false;
    }
    cv::Mat roi = src_img(rect_);
    cv::Mat channels[3];
    cv::split(roi, channels);
    int B_val = (int)(static_cast<int>(mean(channels[0]).val[0]));
    int G_val = (int)(static_cast<int>(mean(channels[1]).val[0]));
    int R_val = (int)(static_cast<int>(mean(channels[2]).val[0]));
    // tools::Tools::drawDiagram("B channel", B_channel_light, armor_config_.window_scale, (B_val * 1.0 / 255), "time", "Avg B", 255, cv::Scalar(255, 0, 0));
    // tools::Tools::drawDiagram("G channel", G_channel_light, armor_config_.window_scale, (G_val * 1.0 / 255), "time", "Avg G", 255, cv::Scalar(0, 255, 0));
    // tools::Tools::drawDiagram("R channel", R_channel_light, armor_config_.window_scale, (R_val * 1.0 / 255), "time", "Avg R", 255, cv::Scalar(0, 0, 255));
    fmt::print("[{}] B {} G {} R {}\n", idntifier_green, B_val, G_val, R_val);
    switch (my_color)
    {
    case uart::RED:
      if (B_val >= image_config_.b_blue_th_min && B_val <= image_config_.b_blue_th_max && G_val >= image_config_.g_blue_th_min && G_val <= image_config_.g_blue_th_max && R_val >= image_config_.r_blue_th_min && R_val <= image_config_.r_blue_th_max)
      {
        fmt::print("[{}] Found blue light\n", idntifier_green);
        return true;
      }
      else
      {
        fmt::print("[{}] Not found blue light\n", idntifier_red);
        return false;
      }
      break;
    case uart::BLUE:
      if (B_val >= image_config_.b_red_th_min && B_val <= image_config_.b_red_th_max && G_val >= image_config_.g_red_th_min && G_val <= image_config_.g_red_th_max && R_val >= image_config_.r_red_th_min && R_val <= image_config_.r_red_th_max)
      {
        fmt::print("[{}] Found red light\n", idntifier_green);
        return true;
      }
      else
      {
        fmt::print("[{}] Not found red light\n", idntifier_red);
        return false;
      }
      break;
    default:
      return false;
      break;
    }
    // cv::copyMakeBorder(channels[0], channels[0], 0, 50, 0, 50, cv::BORDER_CONSTANT, cv::Scalar(0));
    // cv::copyMakeBorder(channels[1], channels[1], 0, 50, 0, 50, cv::BORDER_CONSTANT, cv::Scalar(0));
    // cv::copyMakeBorder(channels[2], channels[2], 0, 50, 0, 50, cv::BORDER_CONSTANT, cv::Scalar(0));
    //  cv::putText(src_img, "B " + std::to_string(B_val), {20, 20}, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0));
    //  cv::putText(src_img, "G " + std::to_string(B_val), {120, 20}, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0));
    //  cv::putText(src_img, "R " + std::to_string(B_val), {220, 20}, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0));

    // tools::Tools::imWindow("[test B]", channels[0], tools::Tools::FIX_MEDIUM); // 150~180 40  B R
    // tools::Tools::imWindow("[test G]", channels[1], tools::Tools::FIX_MEDIUM); // 100~140 60
    // tools::Tools::imWindow("[test R]", channels[2], tools::Tools::FIX_MEDIUM); // 20~30   100
  }

  bool Detector::runBasicArmor(const cv::Mat &_src_img,
                               const uart::Receive_Data _receive_data)
  {
    // 预处理
    std::string window_name = "basic_armor";
    runImage(_src_img, /*_receive_data.my_color*/ uart::RED);
    draw_img_ = _src_img.clone();
    if (findLight(draw_img_, /*_receive_data.my_color*/ uart::RED))
    {
      if (fittingArmor())
      {
        finalArmor();
        lost_cnt_ = 10;
        // 画图开关
        fmt::print("[{}] Find Object\n", idntifier_green);
        if (armor_config_.armor_draw == 1 ||
            light_config_.light_draw == 1 ||
            armor_config_.armor_edit == 1 ||
            light_config_.light_edit == 1)
        {
          if (armor_config_.debug_mode == 1)
          {
            tools::Tools::imWindow(window_name, draw_img_, armor_config_.window_scale);
            // tools::Tools::drawDiagram("diagram", armor_x_point, armor_config_.window_scale, (armor_data_.armor_rect.center.x * 1.0 / _src_img.cols), "time", "armor_x", _src_img.cols);
          }

          draw_img_ = cv::Mat::zeros(_src_img.size(), CV_8UC3);
        }
        return true;
      }
    }
    if (armor_config_.armor_draw == 1 ||
        light_config_.light_draw == 1 ||
        armor_config_.armor_edit == 1 ||
        light_config_.light_edit == 1)
    {
      if (armor_config_.debug_mode == 1)
      {
        tools::Tools::imWindow(window_name, draw_img_, armor_config_.window_scale);
      }
      draw_img_ = cv::Mat::zeros(_src_img.size(), CV_8UC3);
    }
    return false;
  }

  bool Detector::sentryMode(const cv::Mat &_src_img,
                            const uart::Receive_Data _receive_data)
  {
    std::string window_name = "sentry_armor";
    if (sentry_cnt_ == 0)
    {
      // 更新哨兵预测方向及预测量
      initialPredictionData(_receive_data.pitch,
                            _receive_data.bullet_velocity,
                            _receive_data.yaw);
      runImage(_src_img, /*_receive_data.my_color*/ uart::BLUE);
      draw_img_ = _src_img.clone();
      if (findLight(draw_img_, /*_receive_data.my_color*/ uart::BLUE))
      {
        if (fittingArmor())
        {
          finalArmor();
          lost_cnt_ = 10;
          // 限制最大预测位置
          if (abs(forecast_pixels_) >
              armor_[0].armor_rect.size.width * forecast_max_size_ * 0.1)
          {
            forecast_pixels_ =
                armor_[0].armor_rect.size.width * forecast_max_size_ * 0.1;
          }
          // 添加预测值
          if (filter_direction_ > 0.3)
          {
            armor_[0].armor_rect.center.x -= abs(forecast_pixels_);
          }
          else if (filter_direction_ < -0.3)
          {
            armor_[0].armor_rect.center.x += abs(forecast_pixels_);
          }
          last_armor_rect_ = armor_[0].armor_rect;
          cv::rectangle(draw_img_, armor_[0].armor_rect.boundingRect(),
                        cv::Scalar(0, 255, 0), 3, 8);
          if (armor_config_.armor_draw == 1 || light_config_.light_draw == 1 ||
              armor_config_.armor_edit == 1 || light_config_.light_edit == 1)
          {
            if (armor_config_.debug_mode == 1)
            {
              tools::Tools::imWindow(window_name, draw_img_, armor_config_.window_scale);
            }
            draw_img_ = cv::Mat::zeros(_src_img.size(), CV_8UC3);
          }
          return true;
        }
      }
      if (armor_config_.armor_draw == 1 ||
          light_config_.light_draw == 1 ||
          armor_config_.armor_edit == 1 ||
          light_config_.light_edit == 1)
      {
        if (armor_config_.debug_mode == 1)
        {
          tools::Tools::imWindow(window_name, draw_img_, armor_config_.window_scale);
        }
        draw_img_ = cv::Mat::zeros(_src_img.size(), CV_8UC3);
      }
      return false;
    }
    else
    {
      // 初始化陀螺仪零点
      if (sentry_cnt_ < 5)
      {
        initial_gyroscope_ += _receive_data.yaw;
        initial_gyroscope_ *= 0.5;
      }
      sentry_cnt_--;
      return false;
    }
  }

  void Detector::initialPredictionData(const float _gyro_speed_data,
                                       const float _bullet_velocity,
                                       const float _yaw_angle)
  {
    if (armor_config_.armor_forecast && !armor_config_.predict_trackbar)
    {
      std::string window_name = {"[basic_armor] sentryMode() -> sentry_trackbar"};
      cv::namedWindow(window_name);
      cv::createTrackbar("proportion_direction_", window_name, &proportion_direction_, 100, NULL);
      cv::createTrackbar("forecast_size_", window_name, &forecast_size_, 10000, NULL);
      cv::createTrackbar("forecast_max_size_", window_name, &forecast_max_size_, 100, NULL);
      cv::createTrackbar("judge_direction_", window_name, &judge_direction_, 100, NULL);
      cv::createTrackbar("abrupt_variable_", window_name, &abrupt_variable_, 500, NULL);
      cv::createTrackbar("Q", window_name, &Q_, 1000, NULL);
      cv::createTrackbar("R", window_name, &R_, 1000, NULL);
      cv::imshow(window_name, sentry_trackbar_);
      armor_config_.predict_trackbar = true;
    }

    num_cnt_++;
    // 隔帧计算
    if (num_cnt_ % 2 == 0)
    {
      // 判断方向
      if (_gyro_speed_data > judge_direction_ * 0.01)
      {
        current_direction_ = 1;
      }
      else if (_gyro_speed_data < -judge_direction_ * 0.01)
      {
        current_direction_ = -1;
      }
      else
      {
        current_direction_ = 0;
      }
      // 延时滤波
      filter_direction_ = (1 - proportion_direction_ * 0.01) * last_direction_ + proportion_direction_ * 0.01 * current_direction_;
      last_direction_ = filter_direction_;
      // 计算偏差角度
      deviation_angle_ = _yaw_angle - initial_gyroscope_;
      if (last_deviation_angle_ != 0)
      {
        deviation_angle_ = (deviation_angle_ + last_deviation_angle_) * 0.5;
      }
      last_deviation_angle_ = deviation_angle_;

      // 计算水平深度
      actual_z_ = sentry_dist_ / cos(deviation_angle_ * CV_PI / 180);

      // 计算实际深度
      actual_depth_ = std::sqrt(actual_z_ * actual_z_ + sentry_height_ * sentry_height_);

      // 计算预测角度 角速度 * 时间
      forecast_angle_ = static_cast<float>(actual_depth_ * 0.001 / _bullet_velocity) * _gyro_speed_data;

      // 计算像素点个数
      forecast_pixels_ = abs(camera_focal_ * tan(forecast_angle_ * CV_PI / 180) * forecast_size_ * 100 / 4.8);

      kalman_.setParam(R_, Q_, 1.0);

      forecast_pixels_ = (last_last_forecast_pixels_ + last_forecast_pixels_ + forecast_pixels_) * 0.333;

      if (forecast_pixels_ - last_forecast_pixels_ > abrupt_variable_)
      {
        forecast_pixels_ = last_forecast_pixels_ + abrupt_variable_;
      }
      else if (forecast_pixels_ - last_forecast_pixels_ < -abrupt_variable_)
      {
        forecast_pixels_ = last_forecast_pixels_ - abrupt_variable_;
      }
      last_last_forecast_pixels_ = last_forecast_pixels_;
      last_forecast_pixels_ = forecast_pixels_;
      forecast_pixels_ = kalman_.run(forecast_pixels_);
    }
    // 计数器归零
    if (num_cnt_ % 10 == 0)
    {
      num_cnt_ = 0;
    }
  }

  void Detector::finalArmor()
  {
    armor_success = true;

    if (armor_.size() == 1)
    {
      fmt::print("[{}] Info, only one armor\n", idntifier_green);
    }
    else
    {
      fmt::print("[{}] Info, multiple armors\n", idntifier_green);
      // 离图像中心点大小排序从小到大
      std::sort(armor_.begin(), armor_.end(),
                [](Armor_Data _a, Armor_Data _b)
                {
                  return _a.distance_center < _b.distance_center;
                });

      if (armor_config_.armor_draw == 1 ||
          armor_config_.armor_edit == 1)
      {
        cv::rectangle(draw_img_, armor_[0].armor_rect.boundingRect(),
                      cv::Scalar(0, 255, 0), 3, 8);
        // cv::circle(draw_img_, armor_[0].armor_rect.center, 4, cv::Scalar(255,0,0));
      }
    }
    fmt::print("[info] height {}\n", armor_[0].armor_rect.size.height);
  }

  bool Detector::fittingArmor()
  {
    if (armor_config_.armor_edit && !armor_config_.armor_trackbar)
    {

      std::string window_name = {"[basic_armor] fittingArmor() -> armor_trackbar"};
      armor_config_.armor_trackbar = true;
      cv::namedWindow(window_name);

      cv::createTrackbar("light_height_aspect_min", window_name,
                         &armor_config_.light_height_ratio_min, 100, NULL);
      cv::createTrackbar("light_height_aspect_max", window_name,
                         &armor_config_.light_height_ratio_max, 100, NULL);
      cv::createTrackbar("light_width_aspect_min", window_name,
                         &armor_config_.light_width_ratio_min, 100, NULL);
      cv::createTrackbar("light_width_ratio_max", window_name,
                         &armor_config_.light_width_ratio_max, 100, NULL);
      cv::createTrackbar("light_y_different", window_name,
                         &armor_config_.light_y_different, 100, NULL);
      cv::createTrackbar("light_height_different", window_name,
                         &armor_config_.light_height_different, 100, NULL);
      cv::createTrackbar("armor_angle_different", window_name,
                         &armor_config_.armor_angle_different, 100, NULL);
      cv::createTrackbar("small_armor_aspect_min", window_name,
                         &armor_config_.small_armor_aspect_min, 100, NULL);
      cv::createTrackbar("armor_type_th", window_name,
                         &armor_config_.armor_type_th, 100, NULL);
      cv::createTrackbar("big_armor_aspect_max", window_name,
                         &armor_config_.big_armor_aspect_max, 100, NULL);

      cv::imshow(window_name, armor_trackbar_);
    }
    for (size_t i = 0; i != light_.size(); ++i)
    {
      for (size_t j = i + 1; j != light_.size(); ++j)
      {
        int light_left = 0, light_right = 0;
        // 区分左右灯条
        if (light_[i].center.x > light_[j].center.x)
        {
          light_left = j;
          light_right = i;
        }
        else
        {
          light_left = i;
          light_right = j;
        }

        armor_data_.left_light = light_[light_left];
        armor_data_.right_light = light_[light_right];
        // 装甲板倾斜弧度
        float error_angle =
            atan((light_[light_right].center.y - light_[light_left].center.y) /
                 (light_[light_right].center.x - light_[light_left].center.x));
        if (error_angle < 20.f)
        {
          armor_data_.tan_angle = atan(error_angle) * 180 / CV_PI;

          // 拟合装甲板条件判断
          if (lightJudge(light_left, light_right))
          {
            std::cout << "pass\n";
            // 装甲板内颜色平均强度
            if (averageColor() < 30)
            {
              // 储存装甲板
              // cv::line(draw_img_, armor_data_.right_light, vertex[(l + 1) % 4], cv::Scalar(0, 255, 255), 3, 8);
              // std::cout << armor_data_.width * 1.0 / armor_data_.height << '\n';

              if (armor_data_.width * 1.0 / armor_data_.height > 1.0 && armor_data_.width * 1.0 / armor_data_.height < 5.0)
              {
                //tools::Tools::drawDiagram("W/H", W_H_armor, armor_config_.window_scale, (armor_data_.width * 1.0 / armor_data_.height / 12.0), "time", "W/H", 12, cv::Scalar(255, 0, 0));
                armor_.push_back(armor_data_);
                if (armor_config_.armor_draw == 1 ||
                    armor_config_.armor_edit == 1)
                {
                  cv::Point2f vertices[4];
                  armor_data_.armor_rect.points(vertices);
                  rectangle(draw_img_, armor_data_.armor_rect.boundingRect(),
                            cv::Scalar(255, 200, 0), 5, 8);
                  cv::putText(draw_img_, std::to_string(int(armor_data_.height * armor_data_.width)), {int(vertices[1].x), int(vertices[1].y - 5)}, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0));
                }
              }
            }
          }
        }
      }
    }
    if (armor_.size() < 1)
    {
      // fmt::print("[{}] Info, armor not found\n", idntifier_green);
      return false;
    }
    return true;
  }

  bool Detector::lightJudge(const int i, const int j)
  {
    armor_data_.left_light_height =
        MAX(light_[i].size.height, light_[i].size.width);
    armor_data_.left_light_width =
        MIN(light_[i].size.height, light_[i].size.width);
    armor_data_.right_light_height =
        MAX(light_[j].size.height, light_[j].size.width);
    armor_data_.right_light_width =
        MIN(light_[j].size.height, light_[j].size.width);
    armor_data_.light_height_aspect =
        armor_data_.left_light_height / armor_data_.right_light_height;
    armor_data_.light_width_aspect =
        armor_data_.left_light_width / armor_data_.right_light_width;

    // 左右灯条高宽比
    if (armor_data_.light_height_aspect < armor_config_.light_height_ratio_max * 0.1 &&
        armor_data_.light_height_aspect > armor_config_.light_height_ratio_min * 0.1 &&
        armor_data_.light_width_aspect < armor_config_.light_width_ratio_max * 0.1 &&
        armor_data_.light_width_aspect > armor_config_.light_width_ratio_min * 0.1)
    {
      armor_data_.height =
          MIN(armor_data_.left_light.size.height, armor_data_.right_light.size.height);
      // 灯条 y 轴位置差
      if (fabs(armor_data_.left_light.center.y - armor_data_.right_light.center.y) <
          armor_data_.height * armor_config_.light_y_different * 0.1)
      {
        // 灯条高度差
        if (fabs(armor_data_.left_light.size.height - armor_data_.right_light.size.height) <
            armor_data_.height * armor_config_.light_height_different * 0.1)
        {
          armor_data_.width =
              getDistance(armor_data_.left_light.center,
                          armor_data_.right_light.center);

          armor_data_.aspect_ratio = armor_data_.width / (MAX(armor_data_.left_light.size.height, armor_data_.right_light.size.height));
          // std::cout << "rat" << armor_data_.aspect_ratio << "\n";
          // 灯条角度差
          if (fabs(armor_data_.left_light.angle - armor_data_.right_light.angle) <
              armor_config_.armor_angle_different * 0.1)
          {

            cv::RotatedRect rects = cv::RotatedRect(
                (armor_data_.left_light.center + armor_data_.right_light.center) / 2,
                cv::Size(armor_data_.width * 0.5, armor_data_.height * 0.5 + 100),
                armor_data_.tan_angle);

            //(armor_data_.left_light.angle + armor_data_.right_light.angle) / 2

            armor_data_.armor_rect = rects;
            // 装甲板保存灯条离中心点的距离
            armor_data_.distance_center =
                getDistance(armor_data_.armor_rect.center,
                            cv::Point(draw_img_.cols, draw_img_.rows));
            // 小装甲板比例范围
            if (armor_data_.aspect_ratio > armor_config_.small_armor_aspect_min * 0.1 && armor_data_.aspect_ratio < armor_config_.armor_type_th * 0.1)
            {
              armor_data_.distinguish = 0;
              return true;
              // 大装甲板比例范围
            }
            else if (armor_data_.aspect_ratio > armor_config_.armor_type_th * 0.1 && armor_data_.aspect_ratio < armor_config_.big_armor_aspect_max * 0.1)
            {
              armor_data_.distinguish = 1;
              return true;
            }
            // return true;
          }
        }
      }
    }

    return false;
  }
  int Detector::averageColor()
  {
    armor_data_.left_light_height =
        MAX(armor_data_.left_light_height, armor_data_.left_light_width);
    armor_data_.left_light_width =
        MIN(armor_data_.left_light_height, armor_data_.left_light_width);
    armor_data_.right_light_height =
        MAX(armor_data_.right_light_height, armor_data_.right_light_width);
    armor_data_.right_light_width =
        MIN(armor_data_.right_light_height, armor_data_.right_light_width);

    cv::RotatedRect rects =
        cv::RotatedRect((armor_data_.left_light.center + armor_data_.right_light.center) / 2,
                        cv::Size(armor_data_.width - (armor_data_.left_light_width + armor_data_.right_light_width),
                                 ((armor_data_.left_light_height + armor_data_.right_light_height) / 2)),
                        armor_data_.tan_angle);
    cv::rectangle(draw_img_, rects.boundingRect(), cv::Scalar(255, 0, 100), 3, 8);
    cv::Point2f vertices[4];
    rects.points(vertices);

    armor_data_.armor_rect = rects;
    cv::Rect _rect = rects.boundingRect();
    // ROI 安全条件
    if (_rect.x <= 0)
    {
      _rect.x = 0;
    }
    if (_rect.y <= 0)
    {
      _rect.y = 0;
    }
    if (_rect.y + _rect.height >= bin_gray_img.rows)
    {
      _rect.height = bin_gray_img.rows - _rect.y;
    }
    if (_rect.x + _rect.width >= bin_gray_img.cols)
    {
      _rect.width = bin_gray_img.cols - _rect.x;
    }
    // 计算颜色平均强度
    static cv::Mat roi = bin_gray_img(_rect);
    int average_intensity = static_cast<int>(mean(roi).val[0]);
    cv::putText(draw_img_, std::to_string(average_intensity), {int(vertices[0].x), int(vertices[0].y + 100)}, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0));
    return average_intensity;
  }

  void Detector::runImage(const cv::Mat &_src_img, const int _my_color)
  {
    // 选择预处理类型（ BGR / HSV ）
    switch (image_config_.method)
    {
    case 0:
      bin_color_img = fuseImage(grayPretreat(_src_img, _my_color),
                                bgrPretreat(_src_img, _my_color), whitePretreat(_src_img));
      break;
    default:
      bin_color_img = fuseImage(grayPretreat(_src_img, _my_color),
                                hsvPretreat(_src_img, _my_color), whitePretreat(_src_img));
      break;
    }
  }

  cv::Mat Detector::fuseImage(const cv::Mat _bin_gray_img, const cv::Mat _bin_color_img, const cv::Mat _white_img)
  {
    cv::bitwise_and(_bin_color_img, _bin_gray_img, _bin_color_img);
    cv::bitwise_and(_bin_color_img, _white_img, _bin_color_img);
    cv::morphologyEx(_bin_color_img, _bin_color_img, cv::MORPH_DILATE, ele_);
    cv::medianBlur(_bin_color_img, _bin_color_img, 5);
    return _bin_color_img;
  }

  inline cv::Mat Detector::whitePretreat(const cv::Mat &_src_img)
  {
    cv::cvtColor(_src_img, gray_white_img_, cv::COLOR_BGR2GRAY);
    cv::threshold(gray_white_img_, white_img_, image_config_.white_armor_color_th, 255, cv::THRESH_BINARY);
    cv::bitwise_not(white_img_, white_img_);
    return white_img_;
  }

  inline cv::Mat Detector::grayPretreat(const cv::Mat &_src_img,
                                        const int _my_color)
  {
    cv::cvtColor(_src_img, gray_img_, cv::COLOR_BGR2GRAY);
    std::string window_name = {"[basic_armor] grayPretreat() -> gray_trackbar"};
    switch (_my_color)
    {
    case uart::RED:
      // my_color 为红色，则处理蓝色的情况
      if (image_config_.gray_edit && !image_config_.gray_trackbar)
      {
        cv::namedWindow(window_name);
        image_config_.gray_trackbar = true;
        cv::createTrackbar("blue_gray_th", window_name,
                           &image_config_.blue_armor_gray_th, 255, NULL);
        cv::imshow(window_name, gray_trackbar_);
      }
      cv::threshold(gray_img_, bin_gray_img, image_config_.blue_armor_gray_th,
                    255, cv::THRESH_BINARY);
      break;
    case uart::BLUE:
      // my_color 为蓝色，则处理红色的情况
      if (image_config_.gray_edit && !image_config_.gray_trackbar)
      {
        cv::namedWindow(window_name);
        image_config_.gray_trackbar = true;
        cv::createTrackbar("red_gray_th", window_name,
                           &image_config_.red_armor_gray_th, 255, NULL);
        cv::imshow(window_name, gray_trackbar_);
      }

      cv::threshold(gray_img_, bin_gray_img, image_config_.red_armor_gray_th,
                    255, cv::THRESH_BINARY);
      break;
    default:
      // my_color 为默认值，则处理红蓝双色的情况
      if (image_config_.gray_edit && !image_config_.gray_trackbar)
      {
        cv::namedWindow(window_name);
        image_config_.gray_trackbar = true;
        cv::createTrackbar("red_gray_th", window_name,
                           &image_config_.red_armor_gray_th, 255, NULL);
        cv::createTrackbar("blue_gray_th", window_name,
                           &image_config_.blue_armor_gray_th, 255, NULL);
        cv::imshow(window_name, gray_trackbar_);
      }

      cv::threshold(gray_img_, bin_red_gray_img,
                    image_config_.red_armor_gray_th, 255, cv::THRESH_BINARY);
      cv::threshold(gray_img_, bin_blue_gray_img,
                    image_config_.blue_armor_gray_th, 255, cv::THRESH_BINARY);
      cv::bitwise_or(bin_red_gray_img, bin_blue_gray_img, bin_gray_img);
      break;
    }

    if (image_config_.gray_edit && armor_config_.debug_mode)
    {
      tools::Tools::imWindow(window_name, bin_gray_img, armor_config_.window_scale);
      // cv::imshow(window_name, bin_gray_img);
      // cv::waitKey(30);
    }
    return bin_gray_img;
  }

  inline cv::Mat Detector::bgrPretreat(const cv::Mat &_src_img, const int _my_color)
  {
    static std::vector<cv::Mat> _split;
    static cv::Mat bin_color_img;
    static cv::Mat bin_color_green_img;

    cv::split(_src_img, _split);

    std::string window_name = {"[basic_armor] brgPretreat() -> color_trackbar"};

    switch (_my_color)
    {
    case uart::RED:
      // my_color 为红色，则处理蓝色的情况
      cv::subtract(_split[0], _split[2], bin_color_img);
      cv::subtract(_split[0], _split[1], bin_color_green_img);

      if (image_config_.color_edit && !image_config_.bgr_trackbar)
      {
        image_config_.bgr_trackbar = true;
        cv::namedWindow(window_name);
        cv::createTrackbar("blue_color_th", window_name,
                           &image_config_.blue_armor_color_th, 255, NULL);
        cv::imshow(window_name, bgr_trackbar_);
      }

      cv::threshold(bin_color_img, bin_color_img, image_config_.blue_armor_color_th, 255, cv::THRESH_BINARY);
      cv::threshold(bin_color_green_img, bin_color_green_img, image_config_.green_armor_color_th, 255, cv::THRESH_BINARY);
      cv::dilate(bin_color_green_img, bin_color_green_img, ele_);
      cv::bitwise_and(bin_color_green_img, bin_color_img, bin_color_img);
      break;
    case uart::BLUE:
      // my_color 为蓝色，则处理红色的情况
      cv::subtract(_split[2], _split[0], bin_color_img);
      cv::subtract(_split[2], _split[1], bin_color_green_img);

      if (image_config_.color_edit && !image_config_.bgr_trackbar)
      {
        image_config_.bgr_trackbar = true;
        cv::namedWindow(window_name);
        cv::createTrackbar("red_color_th", window_name,
                           &image_config_.red_armor_color_th, 255, NULL);
        cv::imshow(window_name, bgr_trackbar_);
      }

      cv::threshold(bin_color_img, bin_color_img, image_config_.red_armor_color_th, 255, cv::THRESH_BINARY);
      cv::threshold(bin_color_green_img, bin_color_green_img, image_config_.green_armor_color_th, 255, cv::THRESH_BINARY);
      cv::dilate(bin_color_green_img, bin_color_green_img, ele_);
      cv::bitwise_and(bin_color_green_img, bin_color_img, bin_color_img);
      break;
    default:
      // my_color 为默认值，则处理红蓝双色的情况
      cv::subtract(_split[2], _split[0], bin_red_color_img);
      cv::subtract(_split[0], _split[2], bin_blue_color_img);
      cv::subtract(_split[2], _split[1], bin_red_green_img);
      cv::subtract(_split[0], _split[1], bin_blue_green_img);

      if (image_config_.color_edit && !image_config_.bgr_trackbar)
      {
        image_config_.bgr_trackbar = true;
        cv::namedWindow(window_name);
        cv::createTrackbar("red_color_th", window_name,
                           &image_config_.red_armor_color_th, 255, NULL);
        cv::createTrackbar("blue_color_th", window_name,
                           &image_config_.blue_armor_color_th, 255, NULL);
        cv::imshow(window_name, bgr_trackbar_);
      }
      cv::threshold(bin_blue_color_img, bin_blue_color_img, image_config_.blue_armor_color_th, 255, cv::THRESH_BINARY);
      cv::threshold(bin_blue_green_img, bin_blue_green_img, image_config_.green_armor_color_th, 255, cv::THRESH_BINARY);
      cv::threshold(bin_red_color_img, bin_red_color_img, image_config_.red_armor_color_th, 255, cv::THRESH_BINARY);
      cv::threshold(bin_red_green_img, bin_red_green_img, image_config_.green_armor_color_th, 255, cv::THRESH_BINARY);
      cv::bitwise_or(bin_blue_color_img, bin_red_color_img, bin_color_img);
      cv::bitwise_or(bin_blue_green_img, bin_red_green_img, bin_color_green_img);
      cv::bitwise_and(bin_color_img, bin_color_green_img, bin_color_img);
      break;
    }

    if (image_config_.color_edit && armor_config_.debug_mode)
    {
      tools::Tools::imWindow(window_name, bin_color_img, armor_config_.window_scale);
      // cv::imshow(window_name, bin_color_img);
      // cv::waitKey(30);
    }
    return bin_color_img;
  }

  inline cv::Mat Detector::hsvPretreat(const cv::Mat &_src_img,
                                       const int _my_color)
  {
    cv::cvtColor(_src_img, hsv_img, cv::COLOR_BGR2HSV_FULL);
    std::string window_name = {"[basic_armor] hsvPretreat() -> hsv_trackbar"};
    switch (_my_color)
    {
    // my_color 为红色，则处理蓝色的情况
    case uart::RED:
      if (image_config_.color_edit && !image_config_.hsv_trackbar)
      {
        image_config_.hsv_trackbar = true;
        cv::namedWindow(window_name);
        cv::createTrackbar("blue_h_min:", window_name,
                           &image_config_.h_blue_min, 255, NULL);
        cv::createTrackbar("blue_h_max:", window_name,
                           &image_config_.h_blue_max, 255, NULL);
        cv::createTrackbar("blue_s_min:", window_name,
                           &image_config_.s_blue_min, 255, NULL);
        cv::createTrackbar("blue_s_max:", window_name,
                           &image_config_.s_blue_max, 255, NULL);
        cv::createTrackbar("blue_v_min:", window_name,
                           &image_config_.v_blue_min, 255, NULL);
        cv::createTrackbar("blue_v_max:", window_name,
                           &image_config_.v_red_max, 255, NULL);
        cv::imshow(window_name, hsv_trackbar_);
      }

      cv::inRange(hsv_img,
                  cv::Scalar(image_config_.h_blue_min,
                             image_config_.s_blue_min,
                             image_config_.v_blue_min),
                  cv::Scalar(image_config_.h_blue_max,
                             image_config_.s_blue_max,
                             image_config_.v_blue_max),
                  bin_color_img);
      break;
    case uart::BLUE:
      // my_color 为蓝色，则处理红色的情况
      if (image_config_.color_edit && !image_config_.hsv_trackbar)
      {
        image_config_.hsv_trackbar = true;
        cv::namedWindow(window_name);
        cv::createTrackbar("red_h_min:", window_name,
                           &image_config_.h_red_min, 255, NULL);
        cv::createTrackbar("red_h_max:", window_name,
                           &image_config_.h_red_max, 255, NULL);
        cv::createTrackbar("red_s_min:", window_name,
                           &image_config_.s_red_min, 255, NULL);
        cv::createTrackbar("red_s_max:", window_name,
                           &image_config_.s_red_max, 255, NULL);
        cv::createTrackbar("red_v_min:", window_name,
                           &image_config_.v_red_min, 255, NULL);
        cv::createTrackbar("red_v_max:", window_name,
                           &image_config_.v_red_max, 255, NULL);
        cv::imshow(window_name, hsv_trackbar_);
      }
      cv::inRange(hsv_img,
                  cv::Scalar(image_config_.h_red_min,
                             image_config_.s_red_min,
                             image_config_.v_red_min),
                  cv::Scalar(image_config_.h_red_max,
                             image_config_.s_red_max,
                             image_config_.v_red_max),
                  bin_color_img);
      break;
    default:
      // my_color 为默认值，则处理红蓝双色的情况
      if (image_config_.color_edit && !image_config_.hsv_trackbar)
      {
        image_config_.hsv_trackbar = true;
        cv::namedWindow(window_name);
        cv::createTrackbar("red_h_min:", window_name,
                           &image_config_.h_red_min, 255, NULL);
        cv::createTrackbar("red_h_max:", window_name,
                           &image_config_.h_red_max, 255, NULL);
        cv::createTrackbar("red_s_min:", window_name,
                           &image_config_.s_red_min, 255, NULL);
        cv::createTrackbar("red_s_max:", window_name,
                           &image_config_.s_red_max, 255, NULL);
        cv::createTrackbar("red_v_min:", window_name,
                           &image_config_.v_red_min, 255, NULL);
        cv::createTrackbar("red_v_max:", window_name,
                           &image_config_.v_red_max, 255, NULL);

        cv::createTrackbar("blue_h_min:", window_name,
                           &image_config_.h_blue_min, 255, NULL);
        cv::createTrackbar("blue_h_max:", window_name,
                           &image_config_.h_blue_max, 255, NULL);
        cv::createTrackbar("blue_s_min:", window_name,
                           &image_config_.s_blue_min, 255, NULL);
        cv::createTrackbar("blue_s_max:", window_name,
                           &image_config_.s_blue_max, 255, NULL);
        cv::createTrackbar("blue_v_min:", window_name,
                           &image_config_.v_blue_min, 255, NULL);
        cv::createTrackbar("blue_v_max:", window_name, &image_config_.v_red_max,
                           255, NULL);

        cv::imshow(window_name, hsv_trackbar_);
      }

      cv::inRange(hsv_img,
                  cv::Scalar(image_config_.h_red_min,
                             image_config_.s_red_min,
                             image_config_.v_red_min),
                  cv::Scalar(image_config_.h_red_max,
                             image_config_.s_red_max,
                             image_config_.v_red_max),
                  bin_red_color_img);

      cv::inRange(hsv_img,
                  cv::Scalar(image_config_.h_blue_min,
                             image_config_.s_blue_min,
                             image_config_.v_blue_min),
                  cv::Scalar(image_config_.h_blue_max,
                             image_config_.s_blue_max,
                             image_config_.v_blue_max),
                  bin_blue_color_img);
      cv::bitwise_or(bin_blue_color_img, bin_red_color_img, bin_color_img);

      break;
    }

    if (image_config_.color_edit && armor_config_.debug_mode)
    {
      tools::Tools::imWindow(window_name, bin_color_img, armor_config_.window_scale);
      // cv::imshow(window_name, bin_color_img);
      // cv::waitKey(30);
    }

    return bin_color_img;
  }

} // namespace basic_armor
