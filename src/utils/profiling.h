#pragma once

#include <string>

#if defined(__linux__)
#include <pthread.h>
#endif

#if defined(LIGHTNING_ENABLE_NVTX_PROFILING)
#include <nvtx3/nvToolsExt.h>
#endif

namespace lightning::profiling {

/// A lightweight named interval displayed on the Nsight Systems timeline.
class ScopedRange {
   public:
    explicit ScopedRange(const char* name) {
#if defined(LIGHTNING_ENABLE_NVTX_PROFILING)
        nvtxRangePushA(name);
#else
        (void)name;
#endif
    }

    explicit ScopedRange(const std::string& name) : ScopedRange(name.c_str()) {}

    ~ScopedRange() {
#if defined(LIGHTNING_ENABLE_NVTX_PROFILING)
        nvtxRangePop();
#endif
    }

    ScopedRange(const ScopedRange&) = delete;
    ScopedRange& operator=(const ScopedRange&) = delete;
};

/// Linux limits thread names to 15 visible bytes plus the terminating null.
inline void SetCurrentThreadName(const std::string& name) {
#if defined(__linux__)
    const std::string short_name = name.substr(0, 15);
    pthread_setname_np(pthread_self(), short_name.c_str());
#else
    (void)name;
#endif
}

}  // namespace lightning::profiling
