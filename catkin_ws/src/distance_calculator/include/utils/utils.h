#ifndef UTILS_H
#define UTILS_H 

#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <sstream>      // std::ostringstream

#include <numeric> // std::iota
#include <vector>


// for time and date
#include <iomanip>
#include <ctime>
#include <sstream>


// print path
#include <unistd.h>
#include <stdio.h>
#include <limits.h>


// #include <algorithm> // std::generate



namespace bipedlab
{
namespace utils
{
    // return a double between [min, max], not [min, max)
    inline double 
    genInclusiveRandomNumber(double min, double max) 
    {
        // run the seed only ONCE
        static bool once = [](){
            srand((unsigned)(time(NULL)));
            return true;
        } ();
        return min + (max - min) * ((double)rand()/RAND_MAX);
        // return  min + (max - min) * drand48();
    }

    // return a double between [min, max), not [min, max]
    inline double 
    genRandomNumber(double min, double max) 
    {
        // run the seed only ONCE
        static bool once = [](){
            srand((unsigned)(time(NULL)));
            return true;
        } ();

        srand48((unsigned)(time(NULL)));
        return  min + (max - min) * drand48();
    }

    template <typename T>
        std::vector<T> genListOfInclusiveRandomNumbers(
            int num_of_numbers, double min, double max)
    {
        std::vector<T> list_of_numbers(num_of_numbers, 0);
        for (T& num : list_of_numbers)
        {
            num = utils::genInclusiveRandomNumber(min, max);
        }
        // std::generate(list_of_numbers.begin(), 
        //               list_of_numbers.end(), 
        //               utils::genInclusiveRandomNumber(min, max));
        return list_of_numbers;
    }

    template <typename T>
    std::vector<T> genListOfNumbers(
            const T& start, 
            const size_t num_of_numbers,
            const T& increment)
    {
        std::vector<T> list_of_numbers(num_of_numbers, 0);
        for (int i = 0; i < num_of_numbers; ++i)
        {
            list_of_numbers[i] = i * increment + start;
        }

        return list_of_numbers;
    }

    inline void 
    pressEnterToContinue() {
        int c;
        signal(SIGINT, [](int signum) {
                std::exit(signum);} // end of lambda expression
        );
        printf("Press [Enter] key to continue.\n");
        while(1)
        {
            char input = getchar();
            if (input == '\n')
                break;
        }; // option TWO to clean stdin
        //getchar(); // wait for ENTER
    }

    template <typename T>
    std::string toStringWithPrecision(const T a_value, const int n = 6)
    {
        std::ostringstream out;
        out.precision(n);
        out << std::fixed << a_value;
        return out.str();
    }


    template <class T>
    double getAverage(const T& this_list)
    {
        double ave = 0;
        for (const auto& element : this_list)
        {
            ave += element;
        }

        return ave / this_list.size();
    }


    inline
    std::string getTimeNDate()
    {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");

        return oss.str();
    }


    inline
    std::string getCurrentDirectory()
    {
        char path[PATH_MAX];
        if (getcwd(path, sizeof(path)) == NULL) {
            perror("getcwd() error");
        }

        // if (getcwd(path, sizeof(path)) != NULL) {
        //     printf("Current working directory : %s\n", path);
        // } else {
        //     perror("getcwd() error");
        // }


        return std::string(path);
    }






} /* general */ 
    
} /* bipedlab */ 
#endif /* ifndef UTILS_H */
