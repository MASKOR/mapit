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


namespace upns
{

#ifdef USE_QT_STRUCTURES
using upnsuint64 = quint64;
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
#endif


}

extern "C"
{

// keep in sync with .proto
enum UpnsLayerType {
  POINTCLOUD2 = 0,
  OCTOMAP = 1,
  OPENVDB = 2,
  LAST_PREDEFINED = 3
};

#define UPNS_MODULE_API_VERSION 1

}

#endif
