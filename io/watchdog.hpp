#ifndef IO__WATCHDOG_HPP
#define IO__WATCHDOG_HPP

#include <chrono>
#include <memory>
#include <string>

#include "communication/WatchdogClient.h"

namespace io
{
// 看门狗封装（原 ArmorDetect_Node 里 WatchdogClient + 3s 喂狗逻辑）
class Watchdog
{
public:
  explicit Watchdog(const std::string & config_path);
  ~Watchdog();
  Watchdog(const Watchdog &) = delete;
  Watchdog & operator=(const Watchdog &) = delete;

  void feed_if_needed();

private:
  std::unique_ptr<WatchdogClient> client_;
  std::chrono::steady_clock::time_point last_feed_;
  long long feed_interval_ms_ = 3000;
  bool inited_ = false;
};

}  // namespace io

#endif  // IO__WATCHDOG_HPP
