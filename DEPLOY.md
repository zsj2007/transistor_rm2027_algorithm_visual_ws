# 上位机部署指南（Linux）

把 `try` 放到上位机 `~/try`，然后按下面步骤装依赖、编译、运行。

## 1. 传输代码

在开发机执行（把 `~/try` 同步到上位机，排除 build 目录）：

```bash
rsync -av --exclude build/ ~/try/ user@上位机IP:~/try/
```

## 2. 安装系统依赖（Ubuntu）

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config \
  libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev \
  nlohmann-json3-dev libtbb-dev libudev-dev \
  libsuitesparse-dev \
  ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev
```

串口权限（操作 /dev/ttyACM* 需要 dialout 组，**改完要重新登录**）：

```bash
sudo usermod -aG dialout $USER
```

## 3. 安装 g2o 和 Sophus（源码编译）

```bash
# Sophus（依赖 Eigen3 + fmt，先装）
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout 1.22.10
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SOPHUS_EXAMPLES=OFF -DBUILD_SOPHUS_TESTS=OFF
make -j$(nproc) && sudo make install
cd ../..

# g2o（依赖 Eigen3 + suitesparse）
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF \
  -DG2O_BUILD_LINKED_APPS=OFF -DG2O_BUILD_PLUGINS_DEFAULT=OFF
make -j$(nproc) && sudo make install
cd ../..

sudo ldconfig
```

## 4. 安装 OpenVINO（本项目 CMake 用 pkg-config 找 openvino）

方法 A（推荐）：去 Intel 官网下载 OpenVINO 2024 Runtime 的 `.deb`（Ubuntu 22.04 amd64），然后：

```bash
sudo apt install -y ./openvino_2024.*_ubuntu22_amd64.deb
# 让 pkg-config 找到 openvino.pc（路径按实际安装版本改）
export PKG_CONFIG_PATH=/opt/intel/openvino_2024*/runtime/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/intel/openvino_2024*/runtime/lib:$LD_LIBRARY_PATH
# 验证
pkg-config --modversion openvino   # 应输出版本号，如 2024.6.0
```

方法 B（快速验证）：`pip install openvino==2024.6.0`，然后找到其 `openvino.pc` 所在目录加入 `PKG_CONFIG_PATH`。

## 5. 编译

```bash
cd ~/try
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

## 6. 配置网卡（GigE 相机）

相机 `cam_ip=192.168.10.10`，上位机网卡固定为 `192.168.10.25/24`（按实际网卡名改 `enpXs0`）：

```bash
sudo ip addr add 192.168.10.25/24 dev enpXs0
# 或 NetworkManager 方式：
# nmcli con mod "Wired" ipv4.addresses 192.168.10.25/24 ipv4.method manual
ping 192.168.10.10   # 能通就说明相机链路 OK
```

## 7. 运行

```bash
cd ~/try

# 实车模式（GigE 相机 + 串口）
./build/infantry configs/infantry.yaml

# 调试版：多打滚动帧率 + 各阶段耗时
./build/infantry_debug configs/infantry.yaml

# 无硬件自检：视频回放（跑 assets/InputVideo/infantry_blue.mp4）
./build/infantry_debug configs/infantry_video.yaml
```

看帧率和各阶段耗时：日志里找 `[FPS]`（每 30 帧一条）和 `Auto Aim Performance`（每 90 帧报告）。

## 常见问题

- `pkg-config --modversion openvino` 报错 → 第 4 步没配对 `PKG_CONFIG_PATH`。
- 编译报找不到 g2o/Sophus → 第 3 步没装好，`sudo ldconfig` 后再 cmake。
- 相机连不上 → 检查网卡 IP、`ping 192.168.10.10`、网线/供电。
- 串口打不开 → 检查 dialout 组、`ls /dev/ttyACM*`。
- 模型路径错误 → 确认从 `~/try` 运行，`assets/models/` 完整。

编译（-j2 保守一点，想快点可以改 -j4）：
cd ~/transistor_rm2027_algorithm_visual_ws
cmake --build build --target infantry_debug -j2
运行（用视频配置）：
cd ~/transistor_rm2027_algorithm_visual_ws
./build/infantry_debug configs/infantry_video.yaml
