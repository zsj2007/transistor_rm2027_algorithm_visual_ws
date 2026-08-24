#ifndef TOOLS__EXITER_HPP
#define TOOLS__EXITER_HPP

namespace tools
{
class Exiter
{
public:
  Exiter();

  bool exit() const;
};

// 供阻塞在等待循环里的代码（如 camera.read）查询，确保 Ctrl+C 能打断等待
bool exitRequested();

}  // namespace tools

#endif  // TOOLS__EXITER_HPP
