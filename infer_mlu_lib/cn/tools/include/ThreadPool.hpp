#ifndef THREAD_POOL_HPP
#define THREAD_POOL_HPP

#include <iostream>
#include <thread> 
#include <string.h>
#include <semaphore.h>
#include "BlockQueue.hpp"

namespace cn
{
    class Job
    {
        private:
            sem_t _finish;
        public:
            Job();
            ~Job();
            void post();
            void join();
            virtual void run()=0;
    };

    class ThreadPool
    {
        
    private:
        std::vector<std::thread*> _threadVec;
        BlockQueue<Job*>* _jobQue;
        void threadRun();
    public:
        ThreadPool(int threadNum=2, int jobQueSize=2, std::string jobQueName="");
        ~ThreadPool();
        void push(Job* job);
        void join();
    };
}

#endif



