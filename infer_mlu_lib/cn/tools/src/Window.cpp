
#include"Window.hpp"
using namespace std;
using namespace cv;
using namespace cn;

Window::Window(int view, int vw, int vh, int chl, int col, string name)
{
    cout<<"Window: "<<this<<endl;
    int w,h;
    if(view<=col)
    {
        w=vw*view;
        h=vh;
    }
    else
    {
        w=vw*col;
        if(view%col==0)
        {
            h=vh*(view/col);
        }
        else
        {
            h=vh*(view/col+1);
        }
    }

    if(chl==1)
    {
        _mat = new cv::Mat(h,w,CV_8UC1);
    }
    else if(chl==3)
    {
         _mat = new cv::Mat(h,w,CV_8UC3);
    }
    else if(chl==4)
    {
         _mat = new cv::Mat(h,w,CV_8UC4);
    }
   
    _name = name;
    cv::namedWindow(_name,cv::WINDOW_NORMAL);
    _thread = new thread(&Window::show,this);
    _threadFinish = false;
    _vw=vw;
    _vh=vh;
    _col=col;
}

Window::~Window()
{
    cout<<"~Window: "<<this<<endl;
    _threadFinish = true;
    _thread->join();
    delete _thread;
    delete _mat;
}

void Window::show()
{
    while(1)
    {
        if(!cvGetWindowHandle(_name.c_str()))
        {
            cout<<"exit"<<endl;
            exit(0);
        }
        if(_threadFinish)
        {
            break;
        }
        cv::imshow(_name,*_mat);
        cv::waitKey(10);
    }
}

void Window::copy(cv::Mat& img, int index)
{
    img.copyTo(_mat->rowRange(_vh*(index/_col),_vh*(index/_col)+_vh).colRange(_vw*(index%_col),_vw*(index%_col)+_vw));
}