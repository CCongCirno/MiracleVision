# MiracleVision

[![CodeFactor](https://img.shields.io/codefactor/grade/github/CCongCirno/MiracleVision?color=green)](https://www.codefactor.io/repository/github/ccongcirno/miraclevision)

A robot vision project for RoboMaster

Iterate from RoboVision_Sirius(ccongcirno)

2025/03/19
v1.0.0

## Document

### 部署

#### 源代码仓库

源代码仓库：[dgsyrc/MiracleVision: A robot vision project for RoboMaster - https://github.com/](https://github.com/dgsyrc/MiracleVision)

#### 必要环境依赖

[opencv(4.5.5)](https://github.com/opencv/opencv)

[opencv_contrib(4.5.5)](https://github.com/opencv/opencv_contrib)

[fmt](https://github.com/fmtlib/fmt)

[mindvision 工业相机 SDK](https://pan.baidu.com/s/1CHb8mEZtElr9zUweLXUcmA) 提取码 rm24

[onnxruntime](https://github.com/microsoft/onnxruntime)

#### 安装

先安装好opencv & opencv_contrib 4.5.5以及onnxruntime

使用下列命令下载项目源码

```
git clone https://github.com/dgsyrc/MiracleVision.git
```

接着在 `MiracleVision/3rdparty` 下执行下列命令下载 `fmt` 库源码

```
git clone https://github.com/fmtlib/fmt.git
```

将下载好的工业相机SDK解压，在SDK文件夹下使用以下命令安装

```
sudo sh install.sh
```

在项目文件夹 `MiracleVision` 下新建文件夹 `build` 

可使用以下命令（在 `MiracleVision` 下使用）

```
mkdir build
```

执行下列命令编译源码

```
cmake ..
make -j12
```

其中 `-j12` 参数可选填，加了更快，但不建议大于12

执行完成后，在 `build` 文件夹下执行以下命令运行

```
sudo ./bin/MiracleVision
```

## serial communication

### receive data

#### table


| 数据位 | 内容                      | 解释                                                         |
| :----- | :------------------------ | :----------------------------------------------------------- |
| 0      | 头帧                      | ‘S’ (0x53)                                                   |
| 1      | 颜色                      | ALL (0), RED (1), BLUE (2)                                   |
| 2      | 模式                      | 0~9 (见表格后注释)                                           |
| 3      | 机器人 ID                 | 英雄 HERO (0), 无人机 UAV (1), 工程机器人 ENGINEERING (2), 步兵 INFANTRY (3), 哨兵 SENTRY (4) |
| 4      | yaw轴陀螺仪低八位         | 二进制数 (换算见注释)，单位：角度                            |
| 5      | yaw轴陀螺仪数据高八位     | 二进制数 (换算见注释)，单位：角度                            |
| 6      | pitch轴陀螺仪低八位       | 二进制数 (换算见注释)，单位：角度                            |
| 7      | pitch轴陀螺仪数据高八位   | 二进制数 (换算见注释)，单位：角度                            |
| 8      | yaw轴陀螺仪加速度低八位   | 二进制数 (换算见注释)，单位：角度                            |
| 9      | yaw轴陀螺仪加速度高八位   | 二进制数 (换算见注释)，单位：角度                            |
| 10     | pitch轴陀螺仪加速度低八位 | 二进制数 (换算见注释)，单位：角度                            |
| 11     | pitch轴陀螺仪加速度高八位 | 二进制数 (换算见注释)，单位：角度                            |
| 12     | 子弹速度                  | 二进制数 (换算见注释)，单位：m/s                             |
| 13     | 尾帧                      | ‘E’ (0x45)                                                   |

#### comment

1.mode:

| 参数 | 枚举类型标识符           | 模式                     |
| ---- | ------------------------ | ------------------------ |
| 0    | DEFAULT_MODE             | 默认模式（基础自瞄模式） |
| 1    | AUTO_AIM                 | 基础自瞄模式             |
| 2    | ENERGY_BUFF              | 能量机关模式             |
| 3    | SENTINEL_AUTONOMOUS_MODE | 哨兵模式                 |
| 4    | CAMERA_CALIBRATION       | 相机标定模式             |

2.二进制数换算：

| 完整数据总位数 | 接收换算                                                   |
| -------------- | ---------------------------------------------------------- |
| 16             | 合并高低八位接收为short (int16_t)类型转换为float类型后/100 |
| 8              | 接收为unsigned char (u_int8_t)类型转换为float类型/10       |

### send data

#### table


| 数据位 | 内容                                  | 解释                                                         |
| :----- | :------------------------------------ | :----------------------------------------------------------- |
| 0      | 头帧                                  | ‘S’ (0x53)                                                   |
| 1      | 装甲板数量 / 是否识别到能量机关装甲板 | 识别到的机器人装甲板数量 / 能量机关：未发现目标 (0) 发现目标 (1) |
| 2      | 开火命令                              | 不开火(0) 开火 (1)                                           |
| 3      | yaw 轴增加量低八位                    | 二进制数 (换算见注释)，单位：角度                            |
| 4      | yaw 轴增加量低高八位                  | 二进制数 (换算见注释)，单位：角度                            |
| 5      | pitch 轴增加量低低八位                | 二进制数 (换算见注释)，单位：角度                            |
| 6      | pitch 轴增加量低高八位                | 二进制数 (换算见注释)，单位：角度                            |
| 7      | 预测坐标x低八位                       | 二进制数 (换算见注释)，单位：像素                            |
| 8      | 预测坐标x高八位                       | 二进制数 (换算见注释)，单位：像素                            |
| 9      | 预测坐标y低八位                       | 二进制数 (换算见注释)，单位：像素                            |
| 10     | 预测坐标y高八位                       | 二进制数 (换算见注释)，单位：像素                            |
| 11     | 深度低八位                            | 二进制数 (换算见注释)                                        |
| 12     | 深度高八位                            | 二进制数 (换算见注释)                                        |
| 13     | CRC 校验位                            | CRC8                                                         |
| 14     | 尾帧                                  | ‘E’ (0x45)                                                   |

#### comment

1.二进制数换算：

| 完整数据总位数       | 发送换算                                                |
| -------------------- | ------------------------------------------------------- |
| 16 (raw & pitch)     | 原始数据*100去尾转换为short (int16_t)类型再拆分高低八位 |
| 16 (预测坐标 & 深度) | 直接拆分高低8位即可                                     |
| 8                    | 原始数据*10去尾转换为unsigned char (u_int8_t)类型       |



## some command

```
cd ***
git add .
git commit -m "commit"
git push -u origin main
```


## log

2024/02/20
beta 0.1

## Todo

chrono::duration_cast
