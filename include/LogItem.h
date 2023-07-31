#pragma once

#include <iostream>
#include <memory> 

#include "LogMessage.h"

class LogItem{
  public:
    std::shared_ptr<LogMessage> msg;
    bool show;
    bool highlight;
  LogItem(){}
  LogItem(std::shared_ptr<LogMessage> msg){
    this->msg = msg;
  }
  
};