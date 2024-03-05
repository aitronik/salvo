
#include <iostream>
#include "impl/ros2_subscriber.h"
#include "salvo.h"

int main(int argc, char * argv[])
{

    Ros2Subscriber data(argc,argv); // Ros subscriber
    
    Salvo log(argc,argv);      // log saver to store the log into a file
    log.listenTo(data);
  
    data.spin();
 
    return 0;
}