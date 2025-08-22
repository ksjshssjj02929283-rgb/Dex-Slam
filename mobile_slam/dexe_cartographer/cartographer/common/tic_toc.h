#ifndef CARTOGRAPHER_COMMON_TIC_TOC_H_
#define CARTOGRAPHER_COMMON_TIC_TOC_H_

#include <ctime>
#include <cstdlib>
#include <chrono>

namespace cartographer {
namespace common {

class TicToc
{
public:
  TicToc()
  {
    tic();
  }

  void tic()
  {
    start = std::chrono::system_clock::now();
  }

  double toc()
  {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

} // namespace common
} // namespace cartographer

#endif // CARTOGRAPHER_COMMON_TIC_TOC_H_
