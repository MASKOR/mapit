/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2016 Tobias Neumann	<t.neumann@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAPIT_GLOBALS_H
#define MAPIT_GLOBALS_H

#include <mapit/typedefs.h>

#if defined(LOG4CPLUS_FOUND) && defined(MAPIT_ENABLE_LOG4CPLUS) && MAPIT_ENABLE_LOG4CPLUS
  #include <log4cplus/logger.h>
  #include <log4cplus/loggingmacros.h>

  #include <log4cplus/configurator.h> // for init_logging
  #include <log4cplus/consoleappender.h> // for init_logging

  #ifdef MAPIT_DEBUG
    #define log_fatal(msg) do{LOG4CPLUS_FATAL(log4cplus::Logger::getInstance("main"), std::string() + "\033[41m" + msg + "\033[0m"); assert(false); break;}while(true)
    #define log_error(msg) do{LOG4CPLUS_ERROR(log4cplus::Logger::getInstance("main"), std::string() + "\033[1;31m" + msg + "\033[0m"); assert(false); break;}while(true)
    #if defined(_MSC_VER) // Microsoft compiler
      #define log_warn(msg) LOG4CPLUS_WARN(log4cplus::Logger::getInstance("main"), std::string() + msg)
      #define log_info(msg) LOG4CPLUS_INFO(log4cplus::Logger::getInstance("main"), std::string() + msg)
    #elif defined(__GNUC__) // GNU compiler
      #define log_warn(msg) LOG4CPLUS_WARN(log4cplus::Logger::getInstance("main"), std::string() + "\033[1;33m" + msg + "\033[0m")
      #define log_info(msg) LOG4CPLUS_INFO(log4cplus::Logger::getInstance("main"), std::string() + "\033[1;32m" + msg + "\033[0m")
    #endif
  #else // MAPIT_DEBUG
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
  #endif // MAPIT_DEBUG
#define mapit_init_logging()     { log4cplus::BasicConfigurator _logconfig_; _logconfig_.configure(); }
#else // LOG4CPLUS_FOUND
  #include <iomanip>
  #ifdef MAPIT_DEBUG
    #define log_fatal(msg) do{ std::cout << std::string() + msg << std::endl; assert(false); break;}while(true)
    #define log_error(msg) do{ std::cout << std::string() + msg << std::endl; assert(true); break;}while(true)
    #if defined(_MSC_VER) // Microsoft compiler
      #define log_warn(msg) (std::cout << std::string() + msg << std::endl)
      #define log_info(msg) (std::cout << std::string() + msg << std::endl)
    #elif defined(__GNUC__) // GNU compiler
      #define log_warn(msg) (std::cout << "\033[1;33m" << std::string() + msg << "\033[0m" << std::endl)
      #define log_info(msg) (std::cout << "\033[1;32m" << std::string() + msg << "\033[0m"  << std::endl)
    #endif
  #else // MAPIT_DEBUG
    #if defined(_MSC_VER) // Microsoft compiler
      #define log_fatal(msg) (std::cout << std::string() + msg << std::endl)
      #define log_error(msg) (std::cout << std::string() + msg << std::endl)
      #define log_warn(msg) (std::cout << std::string() + msg << std::endl)
      #define log_info(msg) (std::cout << std::string() + msg << std::endl)
    #elif defined(__GNUC__) // GNU compiler
      #define log_fatal(msg) (std::cout << "\033[41m" << std::string() + msg << "\033[0m" << std::endl)
      #define log_error(msg) (std::cout << "\033[1;31m" << std::string() + msg << "\033[0m" << std::endl)
      #define log_warn(msg) (std::cout << "\033[1;33m" << std::string() + msg << "\033[0m" << std::endl)
      #define log_info(msg) (std::cout << "\033[1;32m" << std::string() + msg << "\033[0m" << std::endl)
    #endif
  #endif // MAPIT_DEBUG
#define mapit_init_logging()
#endif // LOG4CPLUS_FOUND
#endif
