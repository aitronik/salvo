#pragma once

class LogLevel{
    public:
    
    enum Severity {
        DEBUG=10,
        INFO=20,
        WARN=30,
        ERROR=40,
        FATAL=50,
        EVENT=60,
        STATE=70
    };
    Severity level;
    static const int numLevels=7;
};