#ifndef LOG_MACROS_H_
#define LOG_MACROS_H_

#include <ctime>
#include <string>
#include <iostream>

inline std::string log_ts() {
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
    return buf;
}

// 运行时调试日志开关：g_debug_log 为 true 时打印统计/耗时类调试日志，否则关闭。
// 通过 ROS 参数 "debug_log" 控制（默认 false）。
inline bool g_debug_log = false;

#define LOG_INFO(...)  do { std::cout << "[" << log_ts() << "] [INFO]  "; printf(__VA_ARGS__); std::cout << std::endl; } while(0)
#define LOG_WARN(...)  do { std::cerr << "[" << log_ts() << "] [WARN]  "; fprintf(stderr, __VA_ARGS__); std::cerr << std::endl; } while(0)
#define LOG_ERROR(...) do { std::cerr << "[" << log_ts() << "] [ERROR] "; fprintf(stderr, __VA_ARGS__); std::cerr << std::endl; } while(0)
#define LOG_COUT(msg)  do { std::cout << "[" << log_ts() << "] " << msg << std::endl; } while(0)
// 调试日志：仅在 g_debug_log 开启时输出
#define LOG_DEBUG(...) do { if (g_debug_log) { std::cout << "[" << log_ts() << "] [DEBUG] "; printf(__VA_ARGS__); std::cout << std::endl; } } while(0)

#endif
