# 基于ROS2的机器人语音交互与视觉感知控制系统

A comprehensive ROS 2 based mobile robot system featuring voice interaction and visual control capabilities.

[演示视频](assets/result.mp4){target="_blank"}

（或者直接在assets文件夹里面下载）
<video width="600" controls>
  <source src="./assets/result.mp4" type="video/mp4">
</video>

![](https://github.com/HulinCal/ros2_voice_visual_armer/blob/main/assets/result.mp4){target="_blank"}


## 项目概述

本项目实现了一个完整的移动机器人系统，集成了以下核心功能：

- **语音交互**：语音采集 → 语音转文字(Whisper) → 大模型推理(Llama) → 文字转语音(TTS) → 语音播报
- **视觉感知**：基于 YOLOv8 的目标检测（苹果等物体识别）
- **机械臂控制**：通过串口通信控制机械臂，支持手眼标定
- **智能推理**：多模态大语言模型，支持图像理解和文本对话

## 系统架构

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   语音输入      │───▶│   语音转文字    │───▶│   大模型推理    │
│ (Audio Capturer)│    │   (Whisper STT) │    │   (Llama LLM)  │
└─────────────────┘    └─────────────────┘    └────────┬────────┘
                                                        │
                                                        ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   语音播报      │◀───│   文字转语音    │◀───────────────│   目标检测      │
│ (Audio Player)  │    │   (Paddle TTS)  │    │  (YOLOv8)      │
└─────────────────┘    └─────────────────┘    └────────┬────────┘
                                                        │
                                                        ▼
                                             ┌─────────────────┐
                                             │   机械臂控制    │
                                             │ (Arm Controller)│
                                             └─────────────────┘
```

## 功能包说明

### 核心功能包

| 包名 | 语言 | 功能描述 |
|------|------|----------|
| [whisper_ros](src/whisper_ros/) | C++ | Whisper 语音识别，包含 VAD 检测和 STT 服务 |
| [llama_ros](src/llama_ros/) | C++ | Llama 大语言模型推理，支持多模态图像理解 |
| [paddle_speech_ros](src/paddle_speech_ros/) | C++ | PaddleSpeech 文字转语音服务 |
| [audio_common](src/audio_common/) | C++ | PortAudio 音频采集与播放 |
| [object_detect](src/object_detect/) | C++ | YOLOv8 目标检测（苹果识别），发布 `/apple_pose` |
| [arm_controller](src/arm_controller/) | C++ | 机械臂控制，支持串口通信和运动学逆解 |
| [hand_eye_calibration](src/hand_eye_calibration/) | Python | 手眼标定工具 |
| [qyu_bringup](src/qyu_bringup/) | Python | 系统启动文件集合 |

### 第三方依赖包

| 包名 | 描述 |
|------|------|
| `whisper_onnxruntime_vendor` | Whisper ONNX 运行时支持 |
| `whisper_hfhub_vendor` | HuggingFace 模型下载支持 |
| `whisper_cpp_vendor` | Whisper.cpp 本地推理支持 |
| `llama_ros_msgs` | Llama 动作消息定义 |
| `whisper_msgs` | Whisper 消息定义 |
| `paddle_speech_msgs` | TTS 消息定义 |
| `astra_camera_ros` | 奥比中光深度相机驱动 |

## 主要节点

### 语音交互节点

| 节点名 | 包 | 功能 |
|--------|-----|------|
| `audio_capturer_node` | audio_common | 麦克风音频采集，发布 `/audio/audio` |
| `audio_player_node` | audio_common | 音频播放，订阅 `/audio/out` |
| `silero_vad_node` | whisper_ros | 语音活动检测（VAD） |
| `whisper_server_node` | whisper_ros | Whisper 语音转文字，动作服务 `/whisper/listen` |
| `llama_action_server` | llama_ros | LLM 推理，动作服务 `llama_inference` |
| `tts_node` | paddle_speech_ros | 文字转语音，动作服务 `text_to_speech` |
| `whisper_demo_nonstop_node` | whisper_demos | 串联语音交互全流程的演示节点 |

### 视觉与机械臂节点

| 节点名 | 包 | 功能 |
|--------|-----|------|
| `apple_detect` | object_detect | YOLOv8 苹果检测，发布 `/apple_pose` |
| `arm_manage_server` | arm_controller | 机械臂控制，动作服务 `arm_interface` |
| `calibrator_node` | hand_eye_calibration | 手眼标定，计算相机到机械臂的坐标变换 |

## 话题与服务

### 话题

| 话题名 | 类型 | 发布节点 | 描述 |
|--------|------|----------|------|
| `/audio/audio` | AudioStamped | audio_capturer_node | 采集的音频数据 |
| `/audio/out` | AudioStamped | - | 待播放音频 |
| `/apple_pose` | geometry_msgs/PointStamped | apple_detect | 检测到的苹果在相机坐标系中的位置 |
| `/camera/color/image_raw` | sensor_msgs/Image | 相机驱动 | 彩色图像 |
| `/camera/depth/image_raw` | sensor_msgs/Image | 相机驱动 | 深度图像 |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | 相机驱动 | 相机内参 |

### 动作服务

| 服务名 | 类型 | 节点 | 描述 |
|--------|------|------|------|
| `/whisper/listen` | STT.action | whisper_server_node | 语音转文字 |
| `llama_inference` | Llama.action | llama_action_server | 大语言模型推理 |
| `text_to_speech` | TTS.action | tts_node | 文字转语音 |
| `arm_interface` | ArmInterface.action | arm_manage_server | 机械臂控制命令 |

## 安装

### 系统要求

- Ubuntu 22.04
- ROS 2 Jazzy
- CMake >= 3.8
- C++17 编译器

### 依赖安装

```bash
# 安装 ROS 2 依赖
sudo apt update
sudo apt install -y ros-jazzy-audio-common ros-jazzy-sensor-msgs ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-geometry-msgs

# 安装 yaml-cpp
sudo apt install -y libyaml-cpp-dev

# 安装 OpenCV
sudo apt install -y libopencv-dev

# 安装 PortAudio
sudo apt install -y portaudio19-dev
```

### 模型文件

将模型文件放置在 `/usr/models/` 目录下：

```bash
# Whisper 模型
/usr/models/ggml-base.bin

# Llama 模型
/usr/models/Qwen2.5-VL-7B-Instruct-IQ4_XS.gguf
/usr/models/mmproj-F16.gguf

# YOLOv8 模型
/usr/models/yolov8s.onnx
```

### 编译

```bash
# 进入工作空间
cd ~/qyu_ws

# 编译所有包
colcon build

# 或只编译特定包
colcon build --packages-select whisper_ros
colcon build --packages-select llama_ros
colcon build --packages-select object_detect
colcon build --packages-select arm_controller
```

## 使用

### 启动语音交互系统


```bash
# 方式一：使用启动文件
ros2 launch qyu_bringup bringup.launch.py


# 方式二：手动启动各节点
# 终端1: 启动语音识别
ros2 launch whisper_bringup whisper.launch.py

# 终端2: 启动大模型
ros2 run llama_ros llama_action_server

# 终端3: 启动 TTS
ros2 run paddle_speech_ros tts_node

# 终端4: 启动演示节点
ros2 run whisper_demos whisper_demo_nonstop_node
```

### 启动视觉与机械臂系统

```bash
# 终端1: 启动相机
ros2 launch astra_camera astra.launch.xml

# 终端2: 启动苹果检测
ros2 run object_detect apple_detect

# 终端3: 启动机械臂控制
ros2 run arm_controller arm_manage_server
```

### 执行抓取苹果任务

```bash
# 使用动作客户端发送命令
ros2 action send_goal /arm_interface arm_controller/action/ArmInterface "{command: 'move_apple'}"
```

### 手眼标定

```bash
# 启动标定节点
ros2 run hand_eye_calibration calibrator_node

# 标定结果保存到
# src/hand_eye_calibration/hand_eye_calibration/calibration_result.yaml
```

## 配置

### 串口配置

机械臂连接在 `/dev/ttyUSB0`，波特率 115200。如需修改，编辑 `arm_controller/src/arm_manage_server.cpp`：

```cpp
SerialController serial_controller_("/dev/ttyUSB0", 115200);
```

### 标定结果路径

手眼标定结果默认从以下路径加载：

```cpp
std::string calibration_file = "/home/hl/qyu_ws/src/hand_eye_calibration/hand_eye_calibration/calibration_result.yaml";
```

## 项目结构

```
qyu_ws/
├── src/
│   ├── whisper_ros/           # Whisper 语音识别
│   │   ├── whisper_ros/       # 核心包
│   │   ├── whisper_demos/      # 演示节点
│   │   ├── whisper_bringup/    # 启动文件
│   │   └── whisper_msgs/       # 消息定义
│   ├── llama_ros/             # 大语言模型
│   │   ├── llama_ros/         # 核心包
│   │   └── llama_ros_msgs/    # 消息定义
│   ├── paddle_speech_ros/     # 文字转语音
│   ├── audio_common/          # 音频采集播放
│   ├── object_detect/         # 目标检测
│   ├── arm_controller/       # 机械臂控制
│   ├── hand_eye_calibration/  # 手眼标定
│   ├── astra_camera_ros/      # 相机驱动
│   ├── qyu_bringup/           # 启动文件
│   └── download/              # 第三方库
├── architecture.md            # 详细架构文档
└── README.md                  # 本文件
```

## License

Apache-2.0 License

## 联系方式

- 维护者: hl
- 邮箱: 3352885695@qq.com
