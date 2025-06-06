/**
 * @file fan_armor.hpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief fan armor identification
 * @date 2024-02-20
 * @copyright Copyright (c) 2024 dgsyrc
 *
 */

#pragma once

#include <vector>

#include "module/buff/abstract_object.hpp"

namespace fan_armor
{
  class Detector : public abstract_object::Object
  {
  public:
    Detector();
    ~Detector();

    /**
     * @brief 输入参数
     * @param[in]  _contours        输入轮廓点集
     */
    void inputParams(const std::vector<cv::Point> &_contours);

    /**
     * @brief 显示内轮廓
     * @param[out]  _img     输出图像
     * @note 绿色
     * @note 图例显示在右侧
     */
    void displayFanArmor(cv::Mat &_img);

  private:
    /* anything else */
  };

} // namespace fan_armor
