#pragma once


#define gg_reset            "\033[0m"

#define fg_black   "\033[30m"
#define fg_red     "\033[31m"
#define fg_green   "\033[32m"
#define fg_yellow  "\033[33m"
#define fg_blue    "\033[34m"
#define fg_magenta "\033[35m"
#define fg_cyan    "\033[36m"
#define fg_white   "\033[37m"

#define bg_black   "\033[40m" 
#define bg_red     "\033[41m" 
#define bg_green   "\033[42m" 
#define bg_yellow  "\033[43m" 
#define bg_blue    "\033[44m" 
#define bg_magenta "\033[45m" 
#define bg_cyan    "\033[46m" 
#define bg_white   "\033[47m" 



#define to_fg_black(s)      std::string( std::string(fg_black)   + s + std::string(gg_reset))
#define to_fg_red(s)        std::string( std::string(fg_red)     + s + std::string(gg_reset))
#define to_fg_green(s)      std::string( std::string(fg_green)   + s + std::string(gg_reset))
#define to_fg_yellow(s)     std::string( std::string(fg_yellow)  + s + std::string(gg_reset))
#define to_fg_blue(s)       std::string( std::string(fg_blue)    + s + std::string(gg_reset))
#define to_fg_magenta(s)    std::string( std::string(fg_magenta) + s + std::string(gg_reset))
#define to_fg_cyan(s)       std::string( std::string(fg_cyan)    + s + std::string(gg_reset))
#define to_fg_white(s)      std::string( std::string(fg_white)   + s + std::string(gg_reset))

#define to_bg_black(s)      std::string( std::string(bg_black)   + s + std::string(gg_reset))
#define to_bg_red(s)        std::string( std::string(bg_red)     + s + std::string(gg_reset))
#define to_bg_green(s)      std::string( std::string(bg_green)   + s + std::string(gg_reset))
#define to_bg_yellow(s)     std::string( std::string(bg_yellow)  + s + std::string(gg_reset))
#define to_bg_blue(s)       std::string( std::string(bg_blue)    + s + std::string(gg_reset))
#define to_bg_magenta(s)    std::string( std::string(bg_magenta) + s + std::string(gg_reset))
#define to_bg_cyan(s)       std::string( std::string(bg_cyan)    + s + std::string(gg_reset))
#define to_bg_white(s)      std::string( std::string(bg_white)   + s + std::string(gg_reset))
 
