#ifndef TIMER_HPP
#define TIMER_HPP

#include <iostream>
#include <sys/time.h>

namespace cn
{
    class Timer
    {
    private:
        struct timeval _start, _end;

    public:
        void start();
        float end();

    };
}

#endif
	



	






