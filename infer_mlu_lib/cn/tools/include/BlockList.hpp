#ifndef BLOCKLIST_HPP
#define BLOCKLIST_HPP

#include <string>
#include <iostream>
#include <list>
#include <mutex>
#include <condition_variable>

namespace cn
{
    template<typename T>
    class BlockList
    {
        private:
            std::list<T> _list;
            std::mutex _mutex;
            std::condition_variable_any _notEmpty;
            std::condition_variable_any _notFull;
            int _size;
            std::string _name;

        public:
            BlockList(int size=1, std::string name="");
            void push(const T& x);
            T pop();
            bool full();
            bool empty();
            void lock();
            void unlock();
            bool find(const T& x);
    };

    template<typename T>
    BlockList<T>:: BlockList(int size, std::string name)
    {
        _size = size;
        _name = name;
    }

    template<typename T>
    bool BlockList<T>::full() 
    {
        return _list.size() == _size;
    }

    template<typename T>
    bool BlockList<T>::empty() 
    {
        return _list.size() == 0;
    }

    template<typename T>
    void BlockList<T>::push(const T& x)
    {
        while(full())
        {
            if(_name!="")
            {
                std::cout<<_name<<" is full,push wating..."<<std::endl;
            }
            _notFull.wait(_mutex);
            if(_name!="")
            {
                std::cout<<_name<<" is not full,push running..."<<std::endl;
            }
        }
    _list.push_back(x);
    _notEmpty.notify_one();
    }

    template<typename T>
    T BlockList<T>::pop()
    {
        while(empty())
        {
            if(_name!="")
            {
                std::cout<<_name<<" is empty,pop wating..."<<std::endl;
            }
            _notEmpty.wait(_mutex);
            if(_name!="")
            {
                std::cout<<_name<<" is not empty,pop running..."<<std::endl;
            }
        }
    
    T x = _list.front();
    _list.pop_front();
    _notFull.notify_one();
    return x;
    }

    template<typename T>
    void BlockList<T>::lock()
    {
        _mutex.lock();
    }

    template<typename T>
    void BlockList<T>::unlock()
    {
        _mutex.unlock();
    }

    template<typename T>
    bool BlockList<T>::find(const T& x)
    {
        typename std::list<T>::iterator iter;
        
        for(iter=_list.begin(); iter!=_list.end(); iter++)
        {
            if(*iter==x)
            {
                break;
            }
        }

        if(iter!=_list.end())
        {
            return true;
        }
        else
        {
            return false;
        }

    }
}

#endif