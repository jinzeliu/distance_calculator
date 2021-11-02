#pragma once
#ifndef TIMMING
#define TIMMING 


#include <math.h>
#include <iostream>
#include <chrono>
#include <ctime>


namespace bipedlab
{
namespace timing
{

    inline
    std::clock_t
    getCurrentCPUTime()
    {
        return std::clock();
    }

    inline
    double spendCPUTime(const std::clock_t &t_start, 
                        const std::clock_t t_end = std::clock())
    {
        return (((double) (t_end - t_start))/CLOCKS_PER_SEC);
    }

    inline
    double spendCPUHz(const std::clock_t &t_start, 
                      const std::clock_t t_end = std::clock()){
        return 1.0/spendCPUTime(t_start, t_end);
    }

    inline
    double printSpendCPUHz(
            const std::clock_t &t_start, std::string txt, 
            const std::clock_t t_end = std::clock()){
        std::cout << std::fixed << std::setprecision(2)
                  << txt << spendCPUHz(t_start, t_end) << " [Hz]"  << std::endl;
    }

    inline
    double printSpendCPUHz(
            const std::clock_t &t_start,
            const std::clock_t &t_end = std::clock()){
        std::string text = "CPU time used: ";
        printSpendCPUHz(t_start, text, t_end);
    }



    // std::chrono::steady_clock::time_point clock_start = std::chrono::steady_clock::now();
    // std::chrono::duration<double> duration =
    //     std::chrono::steady_clock::now() - clock_start;
    inline
    std::chrono::steady_clock::time_point
    getCurrentTime()
    {
        return std::chrono::steady_clock::now();
    }
    
    inline
    double spendElapsedTime(
            const std::chrono::steady_clock::time_point &t_start,
            const std::chrono::steady_clock::time_point &t_end = getCurrentTime())
    {
        std::chrono::duration<double> duration = t_end - t_start;
        return duration.count();
    }

    inline
    double spendElapsedTimeMilli(
            const std::chrono::steady_clock::time_point &t_start,
            const std::chrono::steady_clock::time_point &t_end = getCurrentTime())
    {
        return 1e3*spendElapsedTime(t_end, t_start);
    }

    inline
    double spendElapsedHz(
            const std::chrono::steady_clock::time_point &t_start,
            const std::chrono::steady_clock::time_point &t_end = getCurrentTime())
    {
        return 1.0/spendElapsedTime(t_start, t_end);
    }

    inline
    double printSpendElapsedHz(
            const std::chrono::steady_clock::time_point &t_start,
            std::string txt,
            const std::chrono::steady_clock::time_point &t_end = getCurrentTime())
    {
        std::cout << std::fixed << std::setprecision(2)
                  << txt << spendElapsedHz(t_start, t_end) << " [Hz]"  << std::endl;
    }

    inline
    double printSpendElapsedHz(
            const std::chrono::steady_clock::time_point &t_start,
            const std::chrono::steady_clock::time_point &t_end = getCurrentTime())
    {
        std::string text = "Elapsed time: ";
        printSpendElapsedHz(t_start, text, t_end);
    }

        
} /* timing */ 
    
} /* bipedlab */ 

#endif /* ifndef TIMMING */
