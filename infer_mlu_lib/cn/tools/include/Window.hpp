#ifndef WINDOW_HPP
#define WINDOW_HPP

#include <iostream>
#include <thread> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cn
{
    class Window
    {
    public:
        Window(int view, int vw, int vh, int chl=3, int col=2, std::string name="cambricon");
        ~Window();
        void copy(cv::Mat& img, int index=0);

    private:
        int _vw;
        int _vh;
        int _col;
        std::string _name;
        cv::Mat* _mat;
        std::thread* _thread;
        bool _threadFinish;
        void show();
    };
}

#endif

