#ifndef __UPNS_GLOBALS_H
#define __UPNS_GLOBALS_H

#include "upns_typedefs.h"

//#include <boost/archive/binary_iarchive.hpp>
//#include <boost/archive/binary_oarchive.hpp>

#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>
#ifdef UPNS_DEBUG
  #ifdef USE_QT_STRUCTURES
    #define log_fatal(msg) do{LOG4CPLUS_FATAL(log4cplus::Logger::getInstance("main"), std::string() + msg); qassert_x(false, "upns", msg); break;}while(true)
    #define log_error(msg) do{LOG4CPLUS_ERROR(log4cplus::Logger::getInstance("main"), std::string() + msg); qassert_x(false, "upns", msg); break;}while(true)
  #else
    #define log_fatal(msg) do{LOG4CPLUS_FATAL(log4cplus::Logger::getInstance("main"), std::string() + msg); assert(false); break;}while(true)
    #define log_error(msg) do{LOG4CPLUS_ERROR(log4cplus::Logger::getInstance("main"), std::string() + msg); assert(false); break;}while(true)
  #endif
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



#define UPNS_MODULE_API_VERSION 1

#endif
