#ifndef __UPNS_GLOBALS_H
#define __UPNS_GLOBALS_H

#ifdef USE_QT_STRUCTURES
#include <QVector>
#include <QString>
#include <QSharedPointer>
#include <QPair>
#else
#include <vector>
#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>
#endif

#include <log4cplus/logger.h>
#define log_error(msg) log4cplus::Logger::getInstance("mapfileservice").log(log4cplus::ERROR_LOG_LEVEL, std::string() + msg)
#define log_warn(msg) log4cplus::Logger::getInstance("mapfileservice").log(log4cplus::WARN_LOG_LEVEL, std::string() + msg)
#define log_info(msg) log4cplus::Logger::getInstance("mapfileservice").log(log4cplus::INFO_LOG_LEVEL, std::string() + msg)

namespace upns
{

#ifdef USE_QT_STRUCTURES
using upnsuint64 = quint64;
using upnsuint32 = quint32;
using upnsString = QString;
using upnsReal = qreal;
using upnsIStream = std::istream;
using upnsOStream = std::ostream;
template <typename T>
using upnsVec = QVector<T>;
template<typename T>
using upnsSharedPointer = QSharedPointer<T>;
template <typename T1, typename T2>
using upnsPair = QPair<T1, T2>;
#else
using upnsuint64 = long long unsigned int;
using upnsuint32 = unsigned int;
using upnsString = std::string;
using upnsReal = float;
using upnsIStream = std::istream;
using upnsOStream = std::ostream;
template <typename T>
using upnsVec = std::vector<T>;
template<typename T>
using upnsSharedPointer = boost::shared_ptr<T>;
template <typename T1, typename T2>
using upnsPair = std::pair<T1, T2>;

using LockHandle = upnsuint32;

template<class T, class U>
upnsSharedPointer<T> static_pointer_cast(upnsSharedPointer<U> const & r)
{
    return boost::static_pointer_cast<T>(r);
}
#endif

}

extern "C"
{

// keep in sync with .proto
//enum UpnsLayerType {
//  POINTCLOUD2 = 0,
//  OCTOMAP = 1,
//  OPENVDB = 2,
//  LAST_PREDEFINED = 3
//};

#define UPNS_MODULE_API_VERSION 1

}

#endif
