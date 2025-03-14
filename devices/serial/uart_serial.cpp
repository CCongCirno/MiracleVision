/**
 * @file uart_serial.cpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief serial
 * @date 2024-02-20
 * @copyright Copyright (c) 2024 dgsyrc
 *
 */

#include "uart_serial.hpp"

#define RELEASE

namespace uart
{

  SerialPort::SerialPort(std::string _serial_config)
  {
    cv::FileStorage fs_serial(_serial_config, cv::FileStorage::READ);

    fs_serial["PREFERRED_DEVICE"] >> serial_config_.preferred_device;
    fs_serial["SET_BAUDRATE"] >> serial_config_.set_baudrate;
    fs_serial["SHOW_SERIAL_INFORMATION"] >> serial_config_.show_serial_information;

    const char *DeviceName[] = {serial_config_.preferred_device.c_str(), "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3", "/dev/ttyACM0", "/dev/ttyACM1"};

    struct termios newstate;
    bzero(&newstate, sizeof(newstate));

    for (size_t i = 0; i != sizeof(DeviceName) / sizeof(char *); ++i)
    {
      fd = open(DeviceName[i], O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
      if (fd == -1)
      {
        fmt::print("[{}] Open serial device failed: {}\n", idntifier_red, DeviceName[i]);
      }
      else
      {
        fmt::print("[{}] Open serial device success: {}\n", idntifier_green, DeviceName[i]);

        break;
      }
    }

    switch (serial_config_.set_baudrate)
    {
    case 1:
      cfsetospeed(&newstate, B115200);
      cfsetispeed(&newstate, B115200);
      break;
    case 10:
      cfsetospeed(&newstate, B921600);
      cfsetispeed(&newstate, B921600);
      break;
    default:
      cfsetospeed(&newstate, B115200);
      cfsetispeed(&newstate, B115200);
      break;
    }

    newstate.c_cflag |= CLOCAL | CREAD;
    newstate.c_cflag &= ~CSIZE;
    newstate.c_cflag &= ~CSTOPB;
    newstate.c_cflag |= CS8;
    newstate.c_cflag &= ~PARENB;

    newstate.c_cc[VTIME] = 0;
    newstate.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &newstate);
  }

  SerialPort::~SerialPort(void)
  {
    if (!close(fd))
    {
      fmt::print("[{}] Close serial device success: {}\n", idntifier_green, fd);
    }
  }

  void SerialPort::receiveData()
  {
    // get_flag = false;
    //  初始化接收数据为0
    memset(receive_buff_, 0, REC_INFO_LENGTH * 2);
    // 接收数据
    read_message_ = read(fd, receive_buff_temp_, sizeof(receive_buff_temp_));
    for (int i = 0; i != sizeof(receive_buff_temp_); ++i)
    {
      if (receive_buff_temp_[i] == 'S' && receive_buff_temp_[i + sizeof(receive_buff_) - 1] == 'E')
      {
        if (!get_flag)
        {
          get_flag = true;
        }
        if (serial_config_.show_serial_information == 1)
        {
          fmt::print("[{}] receiveData() ->", idntifier_green);
          for (size_t j = 0; j != sizeof(receive_buff_); ++j)
          {
            receive_buff_[j] = receive_buff_temp_[i + j];
            fmt::print(" {:d}", receive_buff_[j]);
          }
          fmt::print("\n");
        }
        else
        {
          for (size_t j = 0; j != sizeof(receive_buff_); ++j)
          {
            receive_buff_[j] = receive_buff_temp_[i + j];
          }
        }
        break;
      }
    }
    tcflush(fd, TCIFLUSH);
  }

  void SerialPort::writeData(const int &data_type,
                             const int &is_shooting,
                             const int16_t &yaw,
                             const int16_t &pitch,
                             const Write_Data::node &cord,
                             const int16_t &depth)
  {
    // getDataForCRC(data_type, is_shooting, _yaw, yaw, _pitch, pitch, depth, predict_cord, predict_length, predict_width);
    // uint8_t CRC = checksumCRC(crc_buff_, sizeof(crc_buff_))
    getDataForSend(data_type, is_shooting, yaw, pitch, cord, depth, 0);
    write_buff_[0] = 'S';
    write_buff_[14] = 'E';
    write_message_ = write(fd, write_buff_, sizeof(write_buff_));
    // memset(write_buff_, 0, sizeof(write_buff_));
    if (serial_config_.show_serial_information == 1)
    {
      yaw_reduction_ = mergeIntoBytes(write_buff_[4], write_buff_[3]);
      pitch_reduction_ = mergeIntoBytes(write_buff_[6], write_buff_[5]);
      cord_reduction_x = mergeIntoBytes(write_buff_[8], write_buff_[7]);
      cord_reduction_y = mergeIntoBytes(write_buff_[10], write_buff_[9]);
      depth_reduction_ = mergeIntoBytes(write_buff_[12], write_buff_[11]);

      fmt::print("[{}] writeData() ->", idntifier_green);
      for (size_t i = 0; i < 3; ++i)
      {
        fmt::print(" {}", write_buff_[i]);
      }
      fmt::print(" {} {} {} {} {}",
                 static_cast<float>(yaw_reduction_) / 100,
                 static_cast<float>(pitch_reduction_) / 100,
                 static_cast<int>(cord_reduction_x),
                 static_cast<int>(cord_reduction_y),
                 static_cast<float>(depth_reduction_));
      for (size_t i = 13; i < 15; ++i)
      {
        fmt::print(" {}", write_buff_[i]);
      }
      fmt::print("\n");

      yaw_reduction_ = 0x0000;
      pitch_reduction_ = 0x0000;
      depth_reduction_ = 0x0000;
      // memset(write_buff_, 0, sizeof(write_buff_));
    }
  }

  void SerialPort::writeData(const Write_Data &_write_data)
  {
    write_data_.data_type = _write_data.data_type > 1 ? 1 : _write_data.data_type;
    write_data_.is_shooting = _write_data.is_shooting;
    write_data_.yaw = _write_data.yaw * 100;
    write_data_.pitch = _write_data.pitch * 100;
    write_data_.cord = _write_data.cord;
    write_data_.depth = 0;

    writeData(write_data_.data_type,
              write_data_.is_shooting,
              write_data_.yaw,
              write_data_.pitch,
              write_data_.cord,
              write_data_.depth);
  }

  void SerialPort::writeData()
  {
    writeData(write_data_.data_type,
              write_data_.is_shooting,
              write_data_.yaw,
              write_data_.pitch,
              write_data_.cord,
              write_data_.depth);
  }

  // this active
  void SerialPort::updataWriteData(const int _data_type,
                                   const int _is_shooting,
                                   const float _yaw,
                                   const float _pitch,
                                   const Write_Data::node _cord,
                                   const int _depth)
  {
    write_data_.data_type = _data_type > 1 ? 1 : _data_type;
    write_data_.is_shooting = _is_shooting;
    /*if(fabs(_yaw) < 2.0)
    {
      write_data_.yaw = 0.0;
    }
    else
    {
      write_data_.yaw    = _yaw * 100;
    }
    if(fabs(_pitch) < 2.0)
    {
      write_data_.pitch = 0.0;
    }
    else
    {
      write_data_.pitch  = -_pitch * 100;
    }*/
    write_data_.yaw = kal.run(_yaw) * 100;
    write_data_.pitch = kal.run(-_pitch) * 100;
    write_data_.cord = _cord;
    write_data_.depth = _depth;
    writeData();
  }

  Write_Data SerialPort::gainWriteData(const int _data_type,
                                       const int _is_shooting,
                                       const float _yaw,
                                       const float _pitch,
                                       const Write_Data::node _cord,
                                       const int _depth)
  {
    Write_Data write_data;
    write_data.data_type = _data_type > 1 ? 1 : _data_type;
    write_data.is_shooting = _is_shooting;
    write_data.yaw = _yaw * 100;
    write_data.pitch = _pitch * 100;
    write_data_.cord = _cord;
    write_data.depth = _depth;
    return write_data;
  }

  uint8_t SerialPort::checksumCRC(unsigned char *buf, uint16_t len)
  {
    uint8_t check = 0;

    while (len--)
    {
      check = CRC8_Table[check ^ (*buf++)];
    }

    return check;
  }

  void SerialPort::getDataForCRC(const int &data_type,
                                 const int &is_shooting,
                                 const int16_t &yaw,
                                 const int16_t &pitch,
                                 const Write_Data::node &cord,
                                 const int16_t &depth)
  {
    crc_buff_[0] = 0x53;
    crc_buff_[1] = static_cast<unsigned char>(data_type);
    crc_buff_[2] = static_cast<unsigned char>(is_shooting);
    crc_buff_[3] = returnLowBit(yaw);
    crc_buff_[4] = returnHighBit(yaw);
    crc_buff_[5] = returnLowBit(pitch);
    crc_buff_[6] = returnHighBit(pitch);
    crc_buff_[7] = returnLowBit(cord.x);
    crc_buff_[8] = returnHighBit(cord.x);
    crc_buff_[9] = returnLowBit(cord.y);
    crc_buff_[10] = returnHighBit(cord.y);
    crc_buff_[11] = returnLowBit(depth);
    crc_buff_[12] = returnHighBit(depth);
  }

  void SerialPort::getDataForSend(const int &data_type,
                                  const int &is_shooting,
                                  const int16_t &yaw,
                                  const int16_t &pitch,
                                  const Write_Data::node &cord,
                                  const int16_t &depth,
                                  const uint8_t &CRC)
  {
    write_buff_[0] = 0x53;
    write_buff_[1] = static_cast<unsigned char>(data_type);
    write_buff_[2] = static_cast<unsigned char>(is_shooting);
    if (data_type == 0)
    {
      write_buff_[3] = 0;
      write_buff_[4] = 0;
      write_buff_[5] = 0;
      write_buff_[6] = 0;
      write_buff_[7] = 0;
      write_buff_[8] = 0;
      write_buff_[9] = 0;
      write_buff_[10] = 0;
      write_buff_[11] = 0;
      write_buff_[12] = 0;
    }
    else
    {
      write_buff_[3] = returnLowBit(yaw);
      write_buff_[4] = returnHighBit(yaw);
      write_buff_[5] = returnLowBit(pitch);
      write_buff_[6] = returnHighBit(pitch);
      // write_buff_[5]  = returnLowBit(0);
      // write_buff_[6]  = returnHighBit(0);
      write_buff_[7] = returnLowBit(cord.x);
      write_buff_[8] = returnHighBit(cord.x);
      write_buff_[9] = returnLowBit(cord.y);
      write_buff_[10] = returnHighBit(cord.y);
      write_buff_[11] = returnLowBit(depth);
      write_buff_[12] = returnHighBit(depth);
    }
    write_buff_[13] = 0; // CRC & 0xff;
    write_buff_[14] = 0x45;
  }

  bool SerialPort::isEmpty()
  {
    if (get_flag)
    {
      receive_data_.old_flag = false;
      get_flag = false;
      return false;
    }
    else
    {
      receive_data_.old_flag = true;
      get_flag = false;
      return true;
    }
  }

  void SerialPort::updateReceiveInformation()
  {
    receiveData();
    if (!isEmpty())
    {
      if (pre_mode != receive_buff_[2])
      {
#ifndef RELEASE
        cv::destroyAllWindows();
#endif
        pre_mode = receive_buff_[2];
      }
    }
    else
    {
      return;
    }
    // receive_buff_[1] = RED;
    switch (receive_buff_[1])
    {
    case RED:
      receive_data_.my_color = RED;
      fmt::print("[{}] My color is RED\n", idntifier_green);
      break;
    case BLUE:
      receive_data_.my_color = BLUE;
      fmt::print("[{}] My color is BLUE\n", idntifier_green);
      break;
    default:
      receive_data_.my_color = ALL;
      fmt::print("[{}] My color is ALL\n", idntifier_green);
      break;
    }
    switch (receive_buff_[2])
    {
    case AUTO_AIM:
      receive_data_.now_run_mode = AUTO_AIM;
      break;
    case ENERGY_BUFF:
      receive_data_.now_run_mode = ENERGY_BUFF;
      break;
    case SENTINEL_AUTONOMOUS_MODE:
      receive_data_.now_run_mode = SENTINEL_AUTONOMOUS_MODE;
      break;
    case CAMERA_CALIBRATION:
      receive_data_.now_run_mode = CAMERA_CALIBRATION;
      break;
    default:
      receive_data_.now_run_mode = AUTO_AIM;
      break;
    }

    switch (receive_buff_[3])
    {
    case HERO:
      receive_data_.my_robot_id = HERO;
      break;
    case ENGINEERING:
      receive_data_.my_robot_id = ENGINEERING;
      break;
    case INFANTRY:
      receive_data_.my_robot_id = INFANTRY;
      break;
    case UAV:
      receive_data_.my_robot_id = UAV;
      break;
    case SENTRY:
      receive_data_.my_robot_id = SENTRY;
      break;
    default:
      receive_data_.my_robot_id = INFANTRY;
      break;
    }

    for (size_t i = 0; i != sizeof(receive_data_.raw_yaw_angle.arr_yaw); ++i)
    {
      receive_data_.raw_yaw_angle.arr_yaw[i] = receive_buff_[4 + i];
    }
    receive_data_.yaw = receive_data_.raw_yaw_angle.yaw / 100.0;
    fmt::print("[{}] yaw:{}\n", idntifier_green, receive_data_.yaw);

    for (size_t i = 0; i != sizeof(receive_data_.raw_pitch_angle.pitch); ++i)
    {
      receive_data_.raw_pitch_angle.arr_pitch[i] = receive_buff_[6 + i];
    }
    receive_data_.pitch = receive_data_.raw_pitch_angle.pitch / 100.0;
    fmt::print("[{}] pitch:{}\n", idntifier_green, receive_data_.pitch);

    for (size_t i = 0; i != sizeof(receive_data_.raw_yaw_velocity.veloctiy); ++i)
    {
      receive_data_.raw_yaw_velocity.arr_yaw_velocity[i] = receive_buff_[8 + i];
    }
    fmt::print("[{}] yaw_velocity:{}\n", idntifier_green, receive_data_.raw_yaw_velocity.veloctiy);
    receive_data_.yaw_velocity = receive_data_.raw_yaw_velocity.veloctiy / 100.0;

    for (size_t i = 0; i != sizeof(receive_data_.raw_pitch_velocity.veloctiy); ++i)
    {
      receive_data_.raw_pitch_velocity.arr_pitch_velocity[i] = receive_buff_[10 + i];
    }
    fmt::print("[{}] pitch_velocity:{}\n", idntifier_green, receive_data_.raw_pitch_velocity.veloctiy);
    receive_data_.pitch_velocity = receive_data_.raw_pitch_velocity.veloctiy / 100.0;

    receive_data_.raw_bullet_velocity.arr_veloctiy = receive_buff_[12];
    if (receive_data_.raw_bullet_velocity.veloctiy == 0)
    {
      receive_data_.raw_bullet_velocity.veloctiy = 155; // default velocity
    }
    fmt::print("[{}] bullet_velocity:{}\n", idntifier_green, receive_data_.raw_bullet_velocity.arr_veloctiy);
    receive_data_.bullet_velocity = receive_data_.raw_bullet_velocity.veloctiy / 10.0;
  }
} // namespace uart
