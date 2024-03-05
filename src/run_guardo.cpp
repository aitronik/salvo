
#include <iostream>
#include <QApplication>

#include "subscription_manager.h"
#include "log_manager.h"
#include "impl/qt_gui.h"

int main(int argc, char * argv[])
{

    LogManager log;      // log manager containing all the data    
       
    QApplication app(argc, argv);
    app.setWindowIcon(QIcon("salvo.png"));
    QtGui n;

    n.addFilterEventSubscriber(&log);
    n.addFilterNodeSubscriber(&log);

    log.notifyLogMessageTo(&n);
    log.notifyLogListTo(&n);
    
  
    SubscriptionManager manager(argc,argv);
    manager.notifyNodeInfoTo(&n);
    manager.notifyLogMessageTo(&log);

    n.addSubscriptionChoiceSubscriber(&manager);

    return app.exec();
}