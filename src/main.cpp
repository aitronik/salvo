
#include <iostream>
#include "data_subscriber.h"
#include "LogManager.h"

#include "ncursesLib.h"

int main(int argc, char * argv[])
{

    DataSubscriber data; // Ros subscriber
   
    LogManager log; // log manager containing all the data
    
    NcursesLib n; // gui    

//    log.listen(data);
    data.attach(&log); // attach the logmanager to the data

    n.listenTo(log);
      
  
    while(data.isRunning());


    return 0;
}