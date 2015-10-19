#ifndef __UPNS_GLOBALS_H
#define __UPNS_GLOBALS_H

#ifdef USE_QT_STRUCTURES
#include <QVector>
#include <QString>
#else
#include <vector>
#include <string>
#endif


namespace upns
{

#ifdef USE_QT_STRUCTURES
template <typename T>
using upnsVec = QVector<T>;
using upnsuint64 = quint64;
using upnsString = QString;
#else
template <typename T>
using upnsVec = std::vector<T>;
using upnsuint64 = long long unsigned int;
using upnsString = std::string;
#endif

}

#endif
