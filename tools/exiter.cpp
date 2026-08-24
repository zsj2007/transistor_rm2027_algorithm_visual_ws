#include "exiter.hpp"

#include <csignal>
#include <stdexcept>

namespace tools
{
bool exit_ = false;
bool exiter_inited_ = false;

Exiter::Exiter()
{
  if (exiter_inited_) throw std::runtime_error("Multiple Exiter instances!");
  // Ctrl+C / Ctrl+Z / kill 都走优雅退出：置 exit_ 后主循环自然收尾，
  // 而不是让 Ctrl+Z 把进程挂起（终端默认行为）或直接硬杀
  std::signal(SIGINT, [](int) { exit_ = true; });
  std::signal(SIGTERM, [](int) { exit_ = true; });
  std::signal(SIGTSTP, [](int) { exit_ = true; });
  exiter_inited_ = true;
}

bool Exiter::exit() const { return exit_; }

bool exitRequested() { return exit_; }

}  // namespace tools
