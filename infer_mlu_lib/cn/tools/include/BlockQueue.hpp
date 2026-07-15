#ifndef BLOCKQUEUE_HPP
#define BLOCKQUEUE_HPP

#include <string>
#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace cn
{
    template<typename T>
    class BlockQueue
    {
        private:
            std::queue<T> _queue;
            std::mutex _mutex;
            std::condition_variable_any _notEmpty;
            std::condition_variable_any _notFull;
            int _size;
            std::string _name;

        public:
            BlockQueue(int size=1, std::string name="");
            void push(const T& x);
            T pop();
            int size();
            bool full();
            bool empty();
            void lock();
            void unlock();
    };

    template<typename T>
    BlockQueue<T>:: BlockQueue(int size, std::string name)
    {
        _size = size;
        _name = name;
    }

    template<typename T>
    int BlockQueue<T>::size() 
    {
        return _queue.size();
    }

    template<typename T>
    bool BlockQueue<T>::full() 
    {
        return _queue.size() == _size;
    }

    template<typename T>
    bool BlockQueue<T>::empty() 
    {
        return _queue.size() == 0;
    }

    template<typename T>
    void BlockQueue<T>::push(const T& x)
    {
        while(full())
        {
            if(_name!="")
            {
                std::cout<<_name<<" is full,push wating..."<<std::endl;
            }
            _notFull.wait(_mutex);
        }
    _queue.push(x);
    _notEmpty.notify_one();
    }

    template<typename T>
    T BlockQueue<T>::pop()
    {
        while(empty())
        {
            if(_name!="")
            {
                std::cout<<_name<<" is empty,pop wating..."<<std::endl;
            }
            _notEmpty.wait(_mutex);
        }
    
    T x = _queue.front();
    _queue.pop();
    _notFull.notify_one();
    return x;
    }

    template<typename T>
    void BlockQueue<T>::lock()
    {
        _mutex.lock();
    }

    template<typename T>
    void BlockQueue<T>::unlock()
    {
        _mutex.unlock();
    }
}

#endif