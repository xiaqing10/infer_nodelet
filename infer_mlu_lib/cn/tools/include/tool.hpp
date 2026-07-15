#ifndef TOOL_HPP
#define TOOL_HPP

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <sys/types.h>
#include <dirent.h>
#include <cstdio>
#include <cstring>
#include <memory>

namespace cn
{
    std::vector<std::string> list_files(std::string dir_path);
    std::string get_filename(std::string dir_file_path);
}
#endif