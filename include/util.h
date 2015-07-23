#ifndef UTIL_H
#define UTIL_H
#include <string>
#include <iostream>
#include <sstream>

namespace util
{
    void printDebug(std::stringstream& debug_msg)
    {
    #ifdef DEBUG
        std::cout << debug_msg.str();
    #endif
        debug_msg.str(std::string());
    }
}

#endif // UTIL_H
