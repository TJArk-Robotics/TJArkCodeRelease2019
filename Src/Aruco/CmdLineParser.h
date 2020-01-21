#pragma once

#include <string>

// class for parsing command line
// operator [](string cmd) return  whether cmd is present //string operator ()(string cmd) return the value as a string:
// -cmd value
class CmdLineParser
{
    int argc;
    char **argv;

  public:
    CmdLineParser(int _argc, char **_argv) : argc(_argc), argv(_argv) {}
    bool operator[](std::string param)
    {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (std::string(argv[i]) == param)
                idx = i;
        return (idx != -1);
    }
    std::string operator()(std::string param, std::string defvalue = "-1")
    {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (std::string(argv[i]) == param)
                idx = i;
        if (idx == -1)
            return defvalue;
        else
            return (argv[idx + 1]);
    }
};