#ifndef __MODULES_TOOLKIT_FUTEX_H__
#define __MODULES_TOOLKIT_FUTEX_H__

#include <climits>
#include <atomic>

#include <errno.h>
#include <linux/futex.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/cdefs.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <time.h>

namespace ehawkeye {
namespace modules  {
namespace toolkit  {

static inline int __futex(volatile void* ftx, int op, int value, const struct timespec* timeout, int bitset) {
    return syscall(__NR_futex, ftx, op, value, timeout, NULL, bitset);
}

static inline int futex_wake(volatile void* ftx, int count) {
    return __futex(ftx, FUTEX_WAKE, count, nullptr, 0);
}

static inline int futex_wait(volatile void* ftx, int value, int timeout_ms) {
    struct timespec timeout {
        .tv_sec  = timeout_ms / 1000,
        .tv_nsec = (timeout_ms % 1000) * 1000000,
    };
    return __futex(ftx, FUTEX_WAIT, value, timeout_ms ? &timeout : nullptr, 0);
}

static inline int futex_wait_ex(volatile void* ftx, bool shared, int value) {
    return __futex(ftx, (shared ? FUTEX_WAIT_BITSET : FUTEX_WAIT_BITSET_PRIVATE), value, nullptr, FUTEX_BITSET_MATCH_ANY);
}

static inline int futex_wake_ex(volatile void* ftx, bool shared, int count) {
    return __futex(ftx, shared ? FUTEX_WAKE : FUTEX_WAKE_PRIVATE, count, nullptr, 0);
}

static inline int __futex_pi_unlock(volatile void* ftx, bool shared) {
  return __futex(ftx, shared ? FUTEX_UNLOCK_PI : FUTEX_UNLOCK_PI_PRIVATE, 0, nullptr, 0);
}

static int __futex_pi_lock_ex(volatile void* ftx, bool shared, bool use_realtime_clock, const timespec* abs_timeout) {return 0;}

static inline int futex_try_lock_timeout(std::atomic<int>* futex, int timeoutms) {
    int expected = 0;
    if (futex->compare_exchange_strong(expected, 1, std::memory_order_acquire) == false) {
        futex_wait(futex, 1, timeoutms);
        futex->store(1, std::memory_order_release);
    }
    return 0;
}

static inline int futex_try_unlock(std::atomic<int>* futex) {
    futex->store(0, std::memory_order_release);
    return futex_wake(futex, 1);
}

class condition_variable {
public:
    void wait(std::atomic<int>* futex) {
        futex->fetch_add(1, std::memory_order_seq_cst);
        futex_wait(futex, futex->load(std::memory_order_seq_cst), 0);
    }

    void notify_one(std::atomic<int>* futex) {
        futex->store(0);
        futex_wake(futex, 1);
    }

    void notify_all(std::atomic<int>* futex) {
        futex->store(0);
        futex_wake(futex, INT_MAX);
    }
};


} // namespace toolkit
} // namespace modules
} // namespace ehawkeye
#endif//__MODULES_TOOLKIT_FUTEX_H__
