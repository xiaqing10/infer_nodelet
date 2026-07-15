#include"ThreadPool.hpp"

using namespace cn;
using namespace std;

Job::Job()
{
    sem_init(&_finish, 0, 0);
}

Job::~Job()
{
    sem_destroy(&_finish);
}

void Job::post()
{
    sem_post(&_finish);
}

void Job::join()
{
    sem_wait(&_finish);
}

ThreadPool::ThreadPool(int threadNum, int jobQueSize, std::string jobQueName)
{
    cout<<"ThreadPool: "<<this<<endl;
    _jobQue = new BlockQueue<Job*>(jobQueSize, jobQueName);
    for(int i=0;i<threadNum;i++)
    {
        _threadVec.push_back(new thread(&ThreadPool::threadRun, this));
    }
}

ThreadPool::~ThreadPool()
{
    for(auto t:_threadVec)
    {
        delete t;
    }
    delete _jobQue;
    cout<<"~ThreadPool: "<<this<<endl;
}

void ThreadPool::threadRun()
{
    Job* job = nullptr;
    while(1)
    {
        _jobQue->lock();
        job=_jobQue->pop();
        _jobQue->unlock();
        if(job==nullptr)
        {
            break;
        }
        job->run();
        job->post();
    }
}

void ThreadPool::push(Job* job)
{
    _jobQue->lock();
    _jobQue->push(job);
    _jobQue->unlock();
}

void ThreadPool::join()
{
    for(int i=0;i<_threadVec.size();i++)
    {
        _jobQue->lock();
        _jobQue->push(nullptr);
        _jobQue->unlock();
    }
    for(auto t:_threadVec)
    {
        t->join();
    }
}