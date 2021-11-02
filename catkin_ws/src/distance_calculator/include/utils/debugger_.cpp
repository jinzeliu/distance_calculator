#include "dubugger.h"

namespace bipedlab
{
namespace debugger
{
    
    // the higher level, the less outputs will be
    // i.e., the lower level, the less important the message is
    // assign higher level if the message is important, 
    template <class T>
    void debugOutput(std::string text, T& value, int level = 0)
    {
        if (level >= DEBUG_LEVEL)
            std::cout << text << value << std::endl;
    }

    void debugTextOutput(std::string text, int level = 0)
    {
        if (level >= DEBUG_LEVEL)
            std::cout << text << std::endl;
    }
    
    void debugTextCerr(std::string text, int level = 0)
    {
        if (level >= DEBUG_LEVEL)
            std::cerr << text << std::endl;
        
    }
    
    template <class T>
    void debugCerr(std::string text, T& value, int level = 0)
    {
        if (level >= DEBUG_LEVEL)
            std::cerr << text << value << std::endl;
        
    }

} /* debugger */ 
} /* bipedlab */ 

