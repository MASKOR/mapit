#ifndef __UPNS_GLOBALS_H
#define __UPNS_GLOBALS_H

#include "upns_typedefs.h"

#ifdef LOG4CPLUS_FOUND
  #include <log4cplus/logger.h>
  #include <log4cplus/loggingmacros.h>

  #ifdef UPNS_DEBUG
    #define log_fatal(msg) do{LOG4CPLUS_FATAL(log4cplus::Logger::getInstance("main"), std::string() + msg); assert(false); break;}while(true)
    #define log_error(msg) do{LOG4CPLUS_ERROR(log4cplus::Logger::getInstance("main"), std::string() + msg); assert(false); break;}while(true)
    #if defined(_MSC_VER) // Microsoft compiler
      #define log_warn(msg) LOG4CPLUS_WARN(log4cplus::Logger::getInstance("main"), std::string() + msg)
      #define log_info(msg) LOG4CPLUS_INFO(log4cplus::Logger::getInstance("main"), std::string() + msg)
    #elif defined(__GNUC__) // GNU compiler
      #define log_warn(msg) LOG4CPLUS_WARN(log4cplus::Logger::getInstance("main"), std::string() + "\033[1;33m" + msg + "\033[0m")
      #define log_info(msg) LOG4CPLUS_INFO(log4cplus::Logger::getInstance("main"), std::string() + "\033[1;32m" + msg + "\033[0m")
    #endif
  #else // UPNS_DEBUG
  #if defined(_MSC_VER) // Microsoft compiler
    #define log_fatal(msg) LOG4CPLUS_FATAL(log4cplus::Logger::getInstance("main"), std::string() + msg)
    #define log_error(msg) LOG4CPLUS_ERROR(log4cplus::Logger::getInstance("main"), std::string() + msg)
    #define log_warn(msg) LOG4CPLUS_WARN(log4cplus::Logger::getInstance("main"), std::string() + msg)
    #define log_info(msg) LOG4CPLUS_INFO(log4cplus::Logger::getInstance("main"), std::string() + msg)
  #elif defined(__GNUC__) // GNU compiler
    #define log_fatal(msg) LOG4CPLUS_FATAL(log4cplus::Logger::getInstance("main"), std::string() + "\033[41m" + msg + "\033[0m")
    #define log_error(msg) LOG4CPLUS_ERROR(log4cplus::Logger::getInstance("main"), std::string() + "\033[1;31m" + msg + "\033[0m")
    #define log_warn(msg) LOG4CPLUS_WARN(log4cplus::Logger::getInstance("main"), std::string() + "\033[1;33m" + msg + "\033[0m")
    #define log_info(msg) LOG4CPLUS_INFO(log4cplus::Logger::getInstance("main"), std::string() + "\033[1;32m" + msg + "\033[0m")
  #endif
  #endif // UPNS_DEBUG
#else // LOG4CPLUS_FOUND
  #include <iomanip>
  #ifdef UPNS_DEBUG
    #define log_fatal(msg) do{ std::cout << std::to_string(msg) << std::endl; assert(false); break;}while(true)
    #define log_error(msg) do{ std::cout << std::to_string(msg) << std::endl; assert(false); break;}while(true)
    #if defined(_MSC_VER) // Microsoft compiler
      #define log_warn(msg) (std::cout << std::to_string(msg) << std::endl)
      #define log_info(msg) (std::cout << std::to_string(msg) << std::endl)
    #elif defined(__GNUC__) // GNU compiler
      #define log_warn(msg) (std::cout << "\033[1;33m" << std::to_string(msg) << "\033[0m" << std::endl)
      #define log_info(msg) (std::cout << "\033[1;32m" << std::to_string(msg) << "\033[0m"  << std::endl)
    #endif
  #else // UPNS_DEBUG
    #if defined(_MSC_VER) // Microsoft compiler
      #define log_fatal(msg) (std::cout << std::to_string(msg) << std::endl)
      #define log_error(msg) (std::cout << std::to_string(msg) << std::endl)
      #define log_warn(msg) (std::cout << std::to_string(msg) << std::endl)
      #define log_info(msg) (std::cout << std::to_string(msg) << std::endl)
    #elif defined(__GNUC__) // GNU compiler
      #define log_fatal(msg) (std::cout << "\033[41m" << std::to_string(msg) << "\033[0m" << std::endl)
      #define log_error(msg) (std::cout << "\033[1;31m" << std::to_string(msg) << "\033[0m" << std::endl)
      #define log_warn(msg) (std::cout << "\033[1;33m" << std::to_string(msg) << "\033[0m" << std::endl)
      #define log_info(msg) (std::cout << "\033[1;32m" << std::to_string(msg) << "\033[0m" << std::endl)
    #endif
  #endif // UPNS_DEBUG
#endif // LOG4CPLUS_FOUND

#endif