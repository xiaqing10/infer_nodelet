
#include"Timer.hpp"
using namespace std;
using namespace cn;

void Timer::start()
{
    gettimeofday(&_start, NULL);
}

float Timer::end()
{
    float time = 0;
    gettimeofday(&_end, NULL);
    time = (1000000*(_end.tv_sec-_start.tv_sec)+_end.tv_usec-_start.tv_usec)/(1000.0);
    return time;//ms
}
