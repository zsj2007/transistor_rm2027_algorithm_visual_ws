#!/usr/bin/env bash
# setup.sh — transistor_rm2027_algorithm_visual_ws 一键依赖安装 + 验证 + 编译
#
# 用法:
#   ./setup.sh           安装缺失依赖并编译（默认 -j2，避免全开编译崩溃）
#   ./setup.sh --check   只验证依赖是否齐全，不安装、不编译
#   JOBS=8 ./setup.sh    指定编译并行度
#
# 说明:
#   - 系统包用 apt（需要 sudo）；
#   - Sophus / g2o 缺失时从源码自动编译安装到 /usr/local；
#   - OpenVINO 缺失时提示用 pip 安装（或自行安装 .deb）；
#   - Ceres + ~/TorqueController 是可选的 torque 通道依赖，缺了也能编译（仅 serial）。

set -uo pipefail

cd "$(dirname "$0")"

CHECK_ONLY=0
JOBS="${JOBS:-2}"
for arg in "$@"; do
  case "$arg" in
    --check) CHECK_ONLY=1 ;;
    -j*) JOBS="${arg#-j}" ;;
  esac
done

info() { printf '\033[1;34m[setup]\033[0m %s\n' "$*"; }
ok()   { printf '\033[1;32m[  OK ]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[ WARN]\033[0m %s\n' "$*"; }
err()  { printf '\033[1;31m[FAIL ]\033[0m %s\n' "$*"; }

MISSING=()

check_pkg() { # check_pkg <pkg-config 名> <显示名>
  if pkg-config --exists "$1" 2>/dev/null; then
    ok "$2 ($(pkg-config --modversion "$1" 2>/dev/null))"
  else
    warn "缺少 $2"
    MISSING+=("$2")
  fi
}

check_file() { # check_file <路径> <显示名>
  if [ -e "$1" ]; then
    ok "$2"
  else
    warn "缺少 $2 ($1)"
    MISSING+=("$2")
  fi
}

# ============ 1. 基础系统依赖（apt） ============
APT_PKGS=(
  build-essential cmake git pkg-config
  libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev
  libyaml-cpp-dev nlohmann-json3-dev libtbb-dev libudev-dev
  libsuitesparse-dev ffmpeg libavcodec-dev libavformat-dev
  libavutil-dev libswscale-dev
)

if [ "$CHECK_ONLY" -eq 0 ]; then
  info "安装系统依赖: ${APT_PKGS[*]}"
  if sudo -n true 2>/dev/null; then
    sudo apt-get update -y && sudo apt-get install -y "${APT_PKGS[@]}"
  else
    sudo apt-get update -y && sudo apt-get install -y "${APT_PKGS[@]}" || \
      warn "apt 安装失败（可能需要密码或网络），继续检测，最后统一报告"
  fi
else
  info "--check 模式：跳过安装"
fi

# ============ 2. OpenVINO ============
if pkg-config --exists openvino 2>/dev/null; then
  ok "OpenVINO $(pkg-config --modversion openvino)"
else
  warn "OpenVINO 未找到"
  if [ "$CHECK_ONLY" -eq 0 ]; then
    read -r -p "[setup] 尝试用 pip 安装 openvino==2024.6.0 ？[y/N] " ans
    if [ "$ans" = "y" ] || [ "$ans" = "Y" ]; then
      python3 -m pip install openvino==2024.6.0 || warn "pip 安装 OpenVINO 失败"
      OV_PC="$(python3 -c "import openvino, os; print(os.path.join(os.path.dirname(openvino.__file__), 'libs', 'pkgconfig'))" 2>/dev/null)"
      if [ -n "$OV_PC" ] && [ -d "$OV_PC" ]; then
        export PKG_CONFIG_PATH="$OV_PC:${PKG_CONFIG_PATH:-}"
        info "已把 OpenVINO pkgconfig 加入 PATH（当前终端有效，建议写入 ~/.bashrc）"
      fi
    fi
  fi
  MISSING+=("OpenVINO")
fi

# ============ 3. Sophus（find_package(Sophus) 需要） ============
if [ -f /usr/local/include/sophus/se3.hpp ] || [ -f /usr/local/share/sophus/cmake/SophusConfig.cmake ]; then
  ok "Sophus"
else
  warn "Sophus 未安装到 /usr/local"
  if [ "$CHECK_ONLY" -eq 0 ]; then
    info "从源码编译安装 Sophus 1.22.10（需要 sudo）"
    TMP_DIR="$(mktemp -d)"
    git clone --depth 1 -b 1.22.10 https://github.com/strasdat/Sophus.git "$TMP_DIR/Sophus"
    cmake -S "$TMP_DIR/Sophus" -B "$TMP_DIR/Sophus/build" -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SOPHUS_EXAMPLES=OFF -DBUILD_SOPHUS_TESTS=OFF
    cmake --build "$TMP_DIR/Sophus/build" -j"$JOBS"
    sudo cmake --install "$TMP_DIR/Sophus/build"
    rm -rf "$TMP_DIR"
  fi
  MISSING+=("Sophus")
fi

# ============ 4. g2o（find_package(g2o) 需要） ============
if [ -d /usr/local/lib/cmake/g2o ]; then
  ok "g2o"
else
  warn "g2o 未安装到 /usr/local"
  if [ "$CHECK_ONLY" -eq 0 ]; then
    info "从源码编译安装 g2o（需要 sudo）"
    TMP_DIR="$(mktemp -d)"
    git clone --depth 1 https://github.com/RainerKuemmerle/g2o.git "$TMP_DIR/g2o"
    cmake -S "$TMP_DIR/g2o" -B "$TMP_DIR/g2o/build" -DCMAKE_BUILD_TYPE=Release \
      -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF \
      -DG2O_BUILD_LINKED_APPS=OFF -DG2O_BUILD_PLUGINS_DEFAULT=OFF
    cmake --build "$TMP_DIR/g2o/build" -j"$JOBS"
    sudo cmake --install "$TMP_DIR/g2o/build"
    rm -rf "$TMP_DIR"
  fi
  MISSING+=("g2o")
fi

# ============ 5. Ceres + TorqueController（可选，torque 通道） ============
if [ -f /usr/lib/cmake/Ceres/CeresConfig.cmake ] || [ -f /usr/local/lib/cmake/Ceres/CeresConfig.cmake ]; then
  ok "Ceres（torque 通道可用）"
else
  warn "Ceres 未找到：torque 通道不可用（serial 通道不受影响）"
fi
if [ -f "TorqueController/include/RobotController.h" ]; then
  ok "TorqueController（仓库内 vendored，torque 通道可用）"
elif [ -f "${TORQUE_CONTROLLER_DIR:-$HOME/TorqueController}/include/RobotController.h" ]; then
  ok "TorqueController 源码存在（\$HOME，torque 通道可用）"
else
  warn "未找到 TorqueController（仓库内或 ~/）：torque 通道不可用（serial 通道不受影响）"
fi

# ============ 6. 相机 SDK（仓库自带） ============
if [ -f io/camera/lib/amd64/libMvCameraControl.so ]; then
  ok "海康相机 SDK（仓库自带）"
else
  warn "缺少相机 SDK 库 io/camera/lib/amd64/libMvCameraControl.so"
  MISSING+=("相机 SDK")
fi

# ============ 7. 其余 pkg-config 依赖验证 ============
check_pkg opencv4      "OpenCV"
check_pkg fmt          "fmt"
check_pkg spdlog       "spdlog"
check_pkg yaml-cpp     "yaml-cpp"
check_pkg eigen3       "Eigen3"
check_pkg nlohmann_json "nlohmann-json"
check_pkg libavcodec   "ffmpeg(libavcodec)"
check_pkg libavformat  "ffmpeg(libavformat)"
check_pkg libavutil    "ffmpeg(libavutil)"
check_pkg libswscale   "ffmpeg(libswscale)"

# ============ 8. 汇总 ============
echo
if [ "${#MISSING[@]}" -eq 0 ]; then
  ok "依赖验证通过"
else
  err "以下依赖仍缺失: ${MISSING[*]}"
  if [ "$CHECK_ONLY" -eq 1 ]; then
    err "请先运行 ./setup.sh 安装依赖，或按 README.md 手动安装"
  else
    err "安装步骤未能补齐，请按 README.md 手动处理"
  fi
  exit 1
fi

if [ "$CHECK_ONLY" -eq 1 ]; then
  info "依赖齐全，可直接编译: cmake --build build -j$JOBS"
  exit 0
fi

# ============ 9. 编译 ============
info "编译（-j$JOBS）"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$JOBS"
ok "编译完成。运行示例:"
echo "  ./build/infantry configs/infantry.yaml          # 实车"
echo "  ./build/infantry_debug configs/infantry_video.yaml  # 视频回放"
echo "  ./build/visualizer configs/infantry_video.yaml      # 可视化"
