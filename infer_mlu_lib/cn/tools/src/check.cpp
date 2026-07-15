#include "check.hpp"

using namespace std;

namespace cn
{
    int check(int ret,const char *const func,const char *const file,int const line)
    {
        if(ret!=0)
        {     
            string error = "ret="+to_string(ret)+";func:"+string(func)+";file="+string(file)+";line="+to_string(line);
            log(error);
        }
        return ret;
    }

    string date_time()
    {
        time_t now = time(0);
        tm *ltm = localtime(&now);
        return to_string(1+ltm->tm_mon)+"/"+to_string(ltm->tm_mday)+"-"+to_string(ltm->tm_hour)+":"+to_string(ltm->tm_min)+":"+to_string(ltm->tm_sec);
    }

    void delay(int time_ms, int time_us)
    {
        usleep(time_ms*1000+time_us);
    }

    mutex log_mutex ;

    void log(string error, string file)
    {
        lock_guard<mutex> locker(log_mutex);

        error = date_time()+": "+error;
        cout<<error<<endl;
        ofstream error_log(file,ios::app);
        if(error_log.is_open())
        {
            error_log << error+"\n";
            error_log.close();
        }
        else
        {
            cout<<"log to file fail!!!"<<error<<endl;
        }
    }
}