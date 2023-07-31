#pragma once

#include <ncurses.h>
 
#include <unordered_map>

#include "Observer.h"
#include "LogList.h"
#include "NodeInfo.h"

class NcursesLib : public Observer<std::shared_ptr<LogItem>> ,public Observer<std::shared_ptr<NodeInfo>>{
    
    WINDOW *win1;
    WINDOW *win2;
    int num;
    int num2;
    const int height = 10;
    const int numLines = 8;
    std::unordered_map<std::string, std::shared_ptr<NodeInfo>>  m_nodes;
    std::unordered_map<std::string, int>  m_ids;

     std::list<std::string>  m_list;

    public:
    NcursesLib(){
        num  = 0;
        num2  = 0;
        // Initialize ncurses
        initscr();
        cbreak(); // Line buffering disabled
        keypad(stdscr, TRUE); // Enable keypad for function keys
        noecho(); // Disable echoing of input

         // Create two windows
        int width, startY, startX;

        //height = 10;
        width = 40;
        startY = (LINES - height) / 2;    // Center the window vertically
        startX = COLS/2 - width;      // Center the window horizontally

        win1 = newwin(height, width, startY, startX);
        win2 = newwin(height, width, startY, startX + width + 1);

        // Draw borders around the windows
        box(win1, 0, 0);
        box(win2, 0, 0);

        // Add strings to windows
        const char* str1 = "Nodo1";
        const char* str1b = "Nodo2";
        const char* str2 = "[Nodo1] [EVENT], Testo ";

        mvwprintw(win1, 1, 1, str1);
        mvwprintw(win1, 2, 1, str1b);
        mvwprintw(win2, 1, 1, str2);

        // Refresh windows to display the borders and text
        refresh();
        wrefresh(win1);
        wrefresh(win2);

        int ch;
        bool highlightWin1 = true;

        // Highlight the initial window
        highlightWindow(win1, true);
    }
    ~NcursesLib(){
        // Clean up
        delwin(win1);
        delwin(win2);
        endwin();
    }
    void listenTo(LogManager &log){
        log.notifyLogListTo(this); 
        log.notifyNodeInfoTo(this);
    }
 
    
    virtual void update(std::shared_ptr<LogItem> newMsg){
        

        std::string message = "[" + newMsg->msg->name + "]: " + newMsg->msg->msg;

        m_list.push_front(message);

        int i = 0;

        for (std::list<std::string>::iterator it = m_list.begin(); it != m_list.end(); ++it){
            if (i >= numLines){
                break;
            }
            const char* str2 = it->c_str();
            mvwprintw(win2, i+1, 1, str2);
            ++i;
        }
        wrefresh(win2);

    //    std::cout << "[Nucurses] ListItem updated, printing to gui" << std::endl;
    }

    virtual void update(std::shared_ptr<NodeInfo> msg){
        
        auto it = m_nodes.find(msg->name);
        if (it == m_nodes.end()){
            m_ids[msg->name] = m_nodes.size()+1; 
        }
        m_nodes[msg->name] = msg;
        
       
        std::string message = "[" + msg->name + "]: " + msg->lastMsg + " - " + std::to_string(num2);
        num2++;
        const char* str2 = message.c_str();
        mvwprintw(win1, m_ids[msg->name], 1, str2);
        wrefresh(win1);

    //    std::cout << "[Nucurses] ListItem updated, printing to gui" << std::endl;
    }


    void highlightWindow(WINDOW* win, bool highlight) {
        if (highlight) {
            wattron(win, A_REVERSE);
        } else {
            wattroff(win, A_REVERSE);
        }
        wrefresh(win);
    }

};