/**
 * @file angle_solve.cpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief calculate angle pitch and yaw
 * @date 2024-02-20
 * @copyright Copyright (c) 2024 dgsyrc
 *
 */

#include "angle_solve.hpp"

namespace angle_solve
{
    void solve::set_config(std::string config_path)
    {
        cv::FileStorage conf(config_path, cv::FileStorage::READ);
        conf["ARMOR_HEIGHT"] >> config.armor_height;
        conf["ARMOR_LENGHT"] >> config.armor_lenght;
        conf["PIC_ARMOR_HEIGHT"] >> config.pic_armor_height;
        conf["PIC_ARMOR_LENGHT"] >> config.pic_armor_lenght;
        conf["PIC_DISTANCE"] >> config.pic_distance;
        conf["ARMOR_DISTANCE"] >> config.armor_distance;
        conf["SPEED_ARG"] >> config.speed_arg;
        conf.release();
    }

    void solve::angleSolve(cv::RotatedRect object, int row, int col, uart::SerialPort &info)
    {
        //
        // 左加右减 上减下加
        target.predict.y = config.pic_distance * config.armor_height / object.size.height;
        target.predict.x = (object.center.x - col / 2.0) * target.predict.y / config.pic_distance;
        target.predict.z = ((row - object.center.y) - row / 2.0) * target.predict.y / config.pic_distance;
        fmt::print("[{}] Distance: {:.2f} Height: {:.2f} object_y: {:.2f}\n", angle_info, target.predict.y, target.predict.z, object.center.y);
        target.time = target.predict.y / ((info.returnReceiveBulletVelocity() * config.speed_arg) * cos(-info.returnReceivePitch() / 180 * PI) * 100);
        target.predict.z = target.predict.z + 0.5 * 9.8 * 100 * target.time * target.time;
        target.yaw = atan(-target.predict.x / target.predict.y) / PI * 180;
        target.pitch = atan(-target.predict.z / sqrt(pow(target.predict.y, 2) + pow(target.predict.x, 2))) / PI * 180;
        fmt::print("[{}] x {:.2f} y {:.2f} z {:.2f} yaw {:.2f} pitch {:.2f}\n", angle_info, target.predict.x, target.predict.y, target.predict.z, target.yaw, target.pitch);
    }

    void solve::angleSolvePPM(cv::RotatedRect object, int row, int col, Tracker::PPM &ppm_x, Tracker::PPM &ppm_y, uart::SerialPort &info)
    {
        //
        // 左加右减 上减下加
        pos_current[0] = object.center.x;
        pos_current[1] = object.center.y;
        pos_infer[0] = ppm_x.forward(pos_last[0], pos[0], pos_current[0]);
        pos_infer[1] = ppm_y.forward(pos_last[1], pos[1], pos_current[1]);
        fmt::print("[{}] ppm: {:d} {:d} obj: {:d} {:d}\n", angle_info, pos_infer[0], pos_infer[1], (int)object.center.x, (int)object.center.y);
        target.predict.y = config.pic_distance * config.armor_height / object.size.height;
        target.predict.x = (pos_infer[0] - col / 2.0) * target.predict.y / config.pic_distance;
        target.predict.z = ((row - pos_infer[1]) - row / 2.0) * target.predict.y / config.pic_distance;
        fmt::print("[{}] Distance: {:.2f} Height: {:.2f} object_y: {:.2f}\n", angle_info, target.predict.y, target.predict.z, object.center.y);
        target.time = target.predict.y / ((info.returnReceiveBulletVelocity() * config.speed_arg) * cos(-info.returnReceivePitch() / 180 * PI) * 100);
        target.predict.z = target.predict.z + 0.5 * 9.8 * 100 * target.time * target.time;
        target.yaw = atan(-target.predict.x / target.predict.y) / PI * 180;
        target.pitch = atan(-target.predict.z / sqrt(pow(target.predict.y, 2) + pow(target.predict.x, 2))) / PI * 180;
        fmt::print("[{}] x {:.2f} y {:.2f} z {:.2f} yaw {:.2f} pitch {:.2f}\n", angle_info, target.predict.x, target.predict.y, target.predict.z, target.yaw, target.pitch);
        pos_last[0] = pos[0];
        pos_last[1] = pos[1];
        pos[0] = pos_current[0];
        pos[1] = pos_current[1];
        fmt::print("[{}] pos_last: {:d} {:d}  pos: {:d} {:d} pos_current: {:d} {:d}\n", angle_info, pos_last[0], pos_last[1], pos[0], pos[1], pos_current[0], pos_current[1]);
    }

    float solve::returnYawAngle()
    {
        if (fabs(target.yaw) < 0.02)
        {
            return 0;

            // return target.yaw;
        }
        else
        {
            return target.yaw;
        }
    }

    float solve::returnPitchAngle()
    {
        if (fabs(target.pitch) < 0.02)
        {
            return 0;
            // return target.pitch;
        }
        else
        {
            return target.pitch;
        }
    }

    cv::Point solve::getPPMTarget()
    {
        return cv::Point(pos_infer[0], pos_infer[1]);
    }

} // namespace angle_solve
