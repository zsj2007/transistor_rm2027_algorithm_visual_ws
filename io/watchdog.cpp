#include "io/watchdog.hpp"

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
Watchdog::Watchdog(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  if (yaml["watchdog_feed_interval_ms"])
    feed_interval_ms_ = yaml["watchdog_feed_interval_ms"].as<long long>();

  client_ = std::make_unique<WatchdogClient>();
  inited_ = client_->init();
  client_->feed();
  last_feed_ = std::chrono::steady_clock::now();

  if (!inited_) tools::logger()->warn("[Watchdog] init failed, running without watchdog");
}

Watchdog::~Watchdog()
{
  if (client_) client_->stop();
}

void Watchdog::feed_if_needed()
{
  if (!inited_ || !client_ || !client_->isAvailable()) return;
  if (std::chrono::steady_clock::now() - last_feed_ >= std::chrono::milliseconds(feed_interval_ms_)) {
    client_->feed();
    last_feed_ = std::chrono::steady_clock::now();
  }
}

}  // namespace io
