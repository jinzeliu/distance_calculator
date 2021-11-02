// #pragma once
#ifndef DEBUGGER_H
#define DEBUGGER_H


#include <iostream>
#include <string>


extern int DEBUG_LEVEL;
enum text_color_t
{
    BLACK = 30, K = 30,
    RED = 31, R = 31,
    GREEN = 32, G = 32,
    YELLOW = 33, Y = 33, 
    BLUE = 34, B = 34,
    MEGENTA = 35, M = 35,
    CYAN = 36, C = 36,
    WHITE = 37, W = 37,

    // bright color
    BRIGHTBLACK = 90, BK = 90,
    BRIGHTRED = 91, BR = 91,
    BRIGHTGREEN = 92, BG = 92,
    BRIGHTYELLOW = 93, BY = 93, 
    BRIGHTBLUE = 94, BB = 94,
    BRIGHTMEGENTA = 95, BM = 95,
    BRIGHTCYAN = 96, BC = 96,
    BRIGHTWHITE = 97, BW = 97,
};


enum text_front_t
{
    NORMAL = 0, N = 0,
    BOLD = 1,
    FAINT = 2, F = 2,
    ITALIC = 3, I = 3,
    SBLINK = 5,
    RBLINK = 6, FBLINK = 6,

};

namespace bipedlab
{
namespace debugger
{
    // template <class T>
    // void debugOutputStream(std::ostream& out, int level = 0)
    // {
    //     if (level >= DEBUG_LEVEL)
    //         std::cout << out << std::endl;
    // }
    
    // the higher level, the less outputs will be
    // i.e., the lower level, the less important the message is
    // assign higher level if the message is important, 
    template <class T>
    void debugOutput(const std::string text, const T& value, int level = 0)
    {
        if (level >= DEBUG_LEVEL)
            std::cout << text << value << std::endl;
    }

    inline 
    void debugTextOutput(std::string text, int level = 0)
    {
        if (level >= DEBUG_LEVEL)
            std::cout << text << std::endl;
    }


    template <class T> 
    void debugColorOutput(const std::string text, 
            const T& value, int level = 0, 
            text_color_t color = RED, text_front_t font = N)
    {
        if (level >= DEBUG_LEVEL)
            std::cout << "\033[" + std::to_string(font) + ";" + 
                         std::to_string(color) + "m" 
                      << text 
                      << value 
                      << "\033[0m"
                      << std::endl;
    }

    inline
    void debugColorTextOutput(const std::string text, 
            int level = 0, text_color_t color = RED, text_front_t font = N)
    {
        if (level >= DEBUG_LEVEL)
            debugColorOutput(text, "", level, color, font);
    }
    

    inline 
    void debugTitleTextOutput(const std::string script, 
            const std::string& title, int level = 0, 
            text_color_t color = RED, text_front_t font = BOLD, 
            char seperator = '=')
    {
        size_t num_seperator = (size_t) std::max((70.0 - title.size()) / 2.0, 2.0);
        if (level >= DEBUG_LEVEL)
            std::cout << "\033[" + std::to_string(font) + ";" + 
                         std::to_string(color) + "m" 
                      << script 
                      << " "
                      << std::string(num_seperator, seperator) 
                      << " "
                      << title 
                      << " "
                      << std::string(num_seperator, seperator) 
                      << "\033[0m"
                      << std::endl;
    }


    // debugger::debugExitColor(error_msg, __LINE__, __FILE__);
    inline 
    void debugExitColor(std::string error_msg, 
            int line_number, std::string file_name,
            int level = 10, text_color_t color = BR, text_front_t font = BOLD)
    {
        if (level >= DEBUG_LEVEL)
        {
            debugger::debugColorOutput(error_msg, "", 10, color, font);
            std::string error_location = "This error occured at " + file_name + 
                ":" + std::to_string(line_number);
            debugger::debugColorOutput(error_location, "", 10, color, font);
            exit(-1);
        }
        
    }



    inline 
    void debugTextCerr(std::string text, int level = 0)
    {
        if (level >= DEBUG_LEVEL)
        {
            std::cerr << text << std::endl;
            exit(-1);
        }
        
    }
    
    template <class T>
    void debugCerr(std::string text, const T& value, int level = 0)
    {
        if (level >= DEBUG_LEVEL)
        {
            std::cerr << text << value << std::endl;
            exit(-1);
        }
        
    }

} /* debugger */ 
} /* bipedlab */ 
#endif /* ifndef DEBUGGER_H */
