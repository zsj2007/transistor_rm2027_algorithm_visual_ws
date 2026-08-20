#include "logger.hpp"

#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <string>

namespace tools
{
std::shared_ptr<spdlog::logger> logger_ = nullptr;

void set_logger()
{
  auto file_name = fmt::format("logs/{:%Y-%m-%d_%H-%M-%S}.log", std::chrono::system_clock::now());
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
  file_sink->set_level(spdlog::level::debug);

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  // 控制台只保留 info 及以上（含 [FPS] 帧率行、错误/警告），
  // 每帧刷屏的 debug 日志（armor_count、aim center yaw pitch 等）只写入文件。
  console_sink->set_level(spdlog::level::info);
  // 控制台用精简格式（不带时间戳），方便在一屏内找到 [FPS] 行；
  // 文件日志保留默认完整格式，便于事后排查。
  console_sink->set_pattern("[%^%l%$] %v");

  logger_ = std::make_shared<spdlog::logger>("", spdlog::sinks_init_list{file_sink, console_sink});
  logger_->set_level(spdlog::level::debug);
  logger_->flush_on(spdlog::level::info);
}

std::shared_ptr<spdlog::logger> logger()
{
  if (!logger_) set_logger();
  return logger_;
}

}  // namespace tools
