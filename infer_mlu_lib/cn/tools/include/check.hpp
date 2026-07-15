#ifndef CHECK_HPP
#define CHECK_HPP

#include <iostream>
#include <string>
#include <fstream>
#include <mutex>
#include <ctime>
#include "unistd.h"

namespace cn
{
    int check(int ret,const char *const func,const char *const file,int const line);
    void log(std::string error, std::string file="./error_log");
    std::string date_time();
    void delay(int time_ms, int time_us=0);
}
#define CN_CHECK(val) cn::check((val), #val, __FILE__, __LINE__)
#endif