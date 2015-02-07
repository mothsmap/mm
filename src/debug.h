#ifndef __debug__hh__
#define __debug__hh__

#include <boost/timer/timer.hpp>
#include <iostream>
#include <string>

#define DebugVerbose 1

class DebugUtility {
private:
    DebugUtility() {}
    ~DebugUtility() {}
    
public:
    enum DebugLevel {
        Error = 1, // 1
        Warning,   // 2
        Normal,    // 3
        Verbose    // 4
    };
    
    static void Print(DebugLevel level, std::string message) {
        switch (level) {
            case Verbose: {
                if (DebugVerbose >= 4) {
                    std::cout << ">>---Verbose:\t" << message << "\t<<" << std::endl;
                }
            }
                break;
            case Normal: {
                if (DebugVerbose >= 3) {
                    std::cout << ">>---Normal:\t" << message << "\t<<" << std::endl;
                }
            }
                break;
            case Warning: {
                if (DebugVerbose >= 2) {
                    std::cout << ">>***Warning:\t" << message << "\t<<" << std::endl;
                }
            }
                break;
            case Error: {
                if (DebugVerbose >= 1) {
                    std::cout << ">>###Error:\t" << message << "\t<<" << std::endl;;
                }
            }
                break;
            default:
                break;
        }
    }
};

#endif
