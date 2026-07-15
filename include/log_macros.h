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

#define LOG_INFO(...)  do { std::cout << "[" << log_ts() << "] [INFO]  "; printf(__VA_ARGS__); std::cout << std::endl; } while(0)
#define LOG_WARN(...)  do { std::cerr << "[" << log_ts() << "] [WARN]  "; fprintf(stderr, __VA_ARGS__); std::cerr << std::endl; } while(0)
#define LOG_ERROR(...) do { std::cerr << "[" << log_ts() << "] [ERROR] "; fprintf(stderr, __VA_ARGS__); std::cerr << std::endl; } while(0)
#define LOG_COUT(msg)  do { std::cout << "[" << log_ts() << "] " << msg << std::endl; } while(0)

#endif
