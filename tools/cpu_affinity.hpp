#ifndef TOOLS__CPU_AFFINITY_HPP
#define TOOLS__CPU_AFFINITY_HPP

// 线程绑核工具（Linux）。
// 用途：混合架构 CPU（P 核 / E 核）上，把 OpenVINO 之外的线程绑到 E 核，
//       OpenVINO 推理线程由 cpu_pinning.yolo_core_type 绑到 P 核（在 OpenvinoInfer 内处理）。
// 用法：
//   1. main 早期调用 tools::cpu_affinity::initFromYaml(yaml)：
//      解析 cpu_pinning 配置，并把主线程绑到 other_cores；
//      之后新建的线程（线程池、流水线、IO 等）默认继承该亲和掩码。
//   2. 线程池 / 流水线线程入口再调用 applyOtherToCurrentThread() 兜底。
//
// 配置示例（configs/infantry.yaml）：
//   cpu_pinning:
//     enabled: true
//     other_cores: "8-15"              # 其余线程绑定的 E 核逻辑编号
//     yolo_core_type: "pcore"          # OpenVINO 用：pcore / ecore / any
//     yolo_enable_cpu_pinning: true
//     yolo_enable_hyper_threading: true

#include <pthread.h>
#include <sched.h>

#include <cstddef>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace tools {
namespace cpu_affinity {

namespace detail {

inline std::vector<int>& otherCpusRef()
{
  static std::vector<int> cpus;
  return cpus;
}

}  // namespace detail

// 解析 "8-15"、"0,2,4"、"0,2,4-7" 这类写法
inline std::vector<int> parseCpuList(const std::string& text)
{
  std::vector<int> result;
  std::string s = text;
  size_t pos = 0;
  while (pos <= s.size()) {
    size_t comma = s.find(',', pos);
    std::string part = s.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
    pos = (comma == std::string::npos) ? s.size() + 1 : comma + 1;
    if (part.empty()) continue;
    size_t dash = part.find('-');
    if (dash == std::string::npos) {
      result.push_back(std::stoi(part));
    } else {
      int lo = std::stoi(part.substr(0, dash));
      int hi = std::stoi(part.substr(dash + 1));
      for (int c = lo; c <= hi; ++c) {
        result.push_back(c);
      }
    }
  }
  return result;
}

// 把当前线程绑到指定的逻辑 CPU 列表
inline bool applyToCurrentThread(const std::vector<int>& cpus)
{
  if (cpus.empty()) return true;
  cpu_set_t set;
  CPU_ZERO(&set);
  for (int c : cpus) {
    if (c >= 0) CPU_SET(c, &set);
  }
  return pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0;
}

inline void setOtherCpus(const std::vector<int>& cpus)
{
  detail::otherCpusRef() = cpus;
}

inline const std::vector<int>& otherCpus()
{
  return detail::otherCpusRef();
}

inline size_t otherCpuCount()
{
  return detail::otherCpusRef().size();
}

// 把当前线程绑到配置的 E 核列表（线程池 / 流水线线程入口调用）
inline bool applyOtherToCurrentThread()
{
  return applyToCurrentThread(detail::otherCpusRef());
}

// 返回当前线程实际允许的逻辑 CPU 列表（如 "0-17"、"8-15"），用于启动日志验证绑核是否生效
inline std::string currentCpusAllowedList()
{
  cpu_set_t set;
  CPU_ZERO(&set);
  if (sched_getaffinity(0, sizeof(set), &set) != 0) {
    return "?";
  }
  std::string out;
  bool first = true;
  int lo = -1;
  int prev = -1;
  for (int c = 0; c < CPU_SETSIZE; ++c) {
    if (CPU_ISSET(c, &set)) {
      if (lo < 0) lo = c;
      prev = c;
    } else if (lo >= 0) {
      if (!first) out += ",";
      out += (prev == lo) ? std::to_string(lo)
                          : std::to_string(lo) + "-" + std::to_string(prev);
      first = false;
      lo = -1;
    }
  }
  if (lo >= 0) {
    if (!first) out += ",";
    out += (prev == lo) ? std::to_string(lo)
                        : std::to_string(lo) + "-" + std::to_string(prev);
  }
  return out.empty() ? "none" : out;
}

// 解析 yaml 的 cpu_pinning 段，并绑定当前线程（main 早期调用一次）
inline bool initFromYaml(const YAML::Node& yaml)
{
  const YAML::Node& pin = yaml["cpu_pinning"];
  if (!pin || !pin.IsMap()) return false;
  bool enabled = pin["enabled"] ? pin["enabled"].as<bool>() : false;
  if (!enabled) return false;
  std::string other = pin["other_cores"] ? pin["other_cores"].as<std::string>() : "";
  if (other.empty()) return false;
  std::vector<int> cpus = parseCpuList(other);
  setOtherCpus(cpus);
  return applyToCurrentThread(cpus);
}

}  // namespace cpu_affinity
}  // namespace tools

#endif  // TOOLS__CPU_AFFINITY_HPP
