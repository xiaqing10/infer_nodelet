#include "tool.hpp"

using namespace std;

namespace cn
{
    vector<string> list_files(string dir_path)
    {
        vector<string> dir_files;
        DIR* dir = opendir(dir_path.c_str());
        struct dirent *file;
        while ((file = readdir(dir)) != NULL)
        {
            if(strcmp(file->d_name,".")==0||strcmp(file->d_name,"..")==0)
            {
                continue;
            }
            dir_files.push_back(dir_path+"/"+file->d_name);
        }
        closedir(dir);
        return dir_files;
    }

    string get_filename(string dir_file_path)
    {
        int pos = dir_file_path.find_last_of( '/' );
        return dir_file_path.substr(pos + 1);
    }
}