#ifndef ROBOT_LOCALIZATION__REALTIME_SCHEDULING_HPP_
#define ROBOT_LOCALIZATION__REALTIME_SCHEDULING_HPP_

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include <sched.h>

namespace robot_localization
{

constexpr int kRealtimePriority = 99;

inline bool ConfigureRealtimeScheduling()
{
  sched_param parameters{};
  parameters.sched_priority = kRealtimePriority;
  if (sched_setscheduler(0, SCHED_FIFO, &parameters) == 0) {
    std::cout << "[robot_localization] SCHED_FIFO/" << kRealtimePriority
              << " enabled using build-installed CAP_SYS_NICE" << std::endl;
    return true;
  }

  std::cerr << "[robot_localization] Failed to enable SCHED_FIFO/"
            << kRealtimePriority << ": " << std::strerror(errno)
            << ". Rebuild with dexe_mobile_slam/build.sh so the installed "
               "node receives cap_sys_nice=ep."
            << std::endl;
  return false;
}

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__REALTIME_SCHEDULING_HPP_
