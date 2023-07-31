#pragma once

#include <iostream>
 
class LogMessage{
  public:
    uint64_t stamp;
    unsigned char level;
    std::string name;
    std::string msg;
    std::string file;
    std::string function;
    std::string line;
    
};