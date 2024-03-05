#pragma once

#include <iostream>
#include <memory>
#include <list>
#include <unordered_map>
#include <vector>


#include <sys/stat.h>
#include <fstream>
#include <chrono>
#include <argparse/argparse.hpp>


#include "core/observer.h" 
#include "core/subject.h"

#include "core/data_subscriber.h"
#include "core/csv_writer.h"

#define ADMISSIBLE_CHARACTER    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ01234567890_"

class Salvo : public Observer<std::shared_ptr<LogMessage>> {
    
    std::string     m_path;
    std::string     m_fileName;

    std::thread     m_flushThread;
    std::mutex      m_flushMutex;

    bool m_threadRunning;
    bool m_rolling;
    int  m_maxLines;

    CsvWriter m_csvWriter;

public: 
    Salvo(int argc, char * argv[]) : m_csvWriter("stamp,level,name,msg,file,function,line"){


        argparse::ArgumentParser program("salvo");
        program.add_argument("--path")
            .help("path folder where to save the log file")
            .default_value(std::string("."));

        program.add_argument("--name")
            .help("name of the file, it will use the date as default")
            .default_value(std::string(""));

        program.add_argument("--rolling")
            .help("name of the file, it will use the date as default")
            .default_value(false)
            .implicit_value(true);
            
        program.add_argument("--maxlines")
            .help("name of the file, it will use the date as default")
            .scan<'d', int>()
            .default_value(1000); 

        


        try {
            program.parse_args(argc, argv);
        }
        catch (const std::exception& err) {
            std::cerr << err.what() << std::endl;
            std::cerr << program;
            std::exit(1);
        }

        std::string name = program.get<std::string>("--name"); 
        m_path           = program.get<std::string>("--path"); 
        m_rolling        = program.get<bool>("--rolling"); 
        m_maxLines       = program.get<int>("--maxlines"); 

        std::cout << "Input path is:     " << m_path     << std::endl;
        std::cout << "Input name is:     " << name     << std::endl;
        std::cout << "Input rolling is:  " << m_rolling  << std::endl;
        std::cout << "Input maxlines is: " << m_maxLines << std::endl;

        // avoid to use rolling option if name is not set
        if (m_rolling && name.empty()) {
            m_rolling = false;
            std::cout << "name is empty, then rolling false" << std::endl;
        }

        // Check if the directory esists
        struct stat sb;
        if (stat(m_path.c_str(), &sb) == 0 && (sb.st_mode & S_IFDIR)){
            std::cout << "The path is valid!" << std::endl;
        }
        else {
            std::cout << "The Path is invalid!"<< std::endl; 
            m_path = ".";
        }
        
        // check name file, if empty, then use the date.
        if(name.compare("") == 0) {
            // compute the string of the current time and date 
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S.csv");

            // create the full path
            m_fileName = m_path  // provided folder 
                + ( m_path.back() == '/' ? "log_" : "/log_" ) // check if there is already a slash
                + ss.str(); // date and extension
        }
        else {
            //check if log name has admissible carachter
            if(name.find_first_not_of(ADMISSIBLE_CHARACTER) != std::string::npos) {
                std::cout << "The Name is invalid! Change to a standard name. " << std::endl; 
                name = "standard_log";
            }
            m_fileName = m_path
                + ( m_path.back() == '/' ? name : "/" + name ) // check if there is already a slash
                + ".csv";
        }
        std::cout << "Save log to " << m_fileName << std::endl;

        // if not rolling, 
        if (!m_rolling) { //open in overwrite mode
            m_csvWriter.open(m_fileName);
        }
        else { // rolling, so open in append mode
            m_csvWriter.append(m_fileName);
        }
        
        m_threadRunning = true;
        m_flushThread = std::thread(&Salvo::threadFun,this);
    }

    ~Salvo(){
        std::cout << "Saving File to disk "<< std::endl;
        m_threadRunning = false;
        if (m_flushThread.joinable()){
            m_flushThread.join();
        }
        m_csvWriter.close();        
    }
 
    void listenTo(DataSubscriber &data){
        data.addLogMessageSubscriber(this);
    }
  
    virtual void update(std::shared_ptr<LogMessage> msg){
   
        m_flushMutex.lock();
        m_csvWriter.write(msg);
        m_flushMutex.unlock();
    } 

    void threadFun(){
        
        while (m_threadRunning){
            m_flushMutex.lock();
            if (m_rolling) {
                m_csvWriter.limitLines(m_maxLines);
            }
            m_csvWriter.flush();
            m_flushMutex.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

};