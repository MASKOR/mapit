#ifndef __UPNS_TYPEDEFS_H
#define __UPNS_TYPEDEFS_H

#include <cstddef> // for NULL, size_t, ...
#ifdef USE_QT_STRUCTURES
  #include <QVector>
  #include <QString>
  #include <QSharedPointer>
  #include <QPair>
#else
  #include <vector>
  #include <string>
  #include <iostream>
  #ifdef USE_BOOST_STRUCTURES
    #include <boost/shared_ptr.hpp>
  #else
    #include <memory>
  #endif
#endif

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
typedef long long unsigned int upnsuint64;
typedef unsigned int upnsuint32;
typedef std::string upnsString;
typedef float upnsReal;
typedef std::istream upnsIStream;
typedef std::ostream upnsOStream;
//typedef boost::archive::binary_iarchive upnsIStream;
//typedef boost::archive::binary_oarchive upnsOStream;


// woha! works since VC 2013...
template <typename T>
using upnsVec = std::vector<T>;

#ifdef USE_BOOST_STRUCTURES
template<typename T>
using upnsSharedPointer = boost::shared_ptr<T>;
template<class T, class U>
upnsSharedPointer<T> static_pointer_cast(upnsSharedPointer<U> const & r)
{
    return boost::static_pointer_cast<T>(r);
}
#else
template<typename T>
using upnsSharedPointer = std::shared_ptr<T>;
template<class T, class U>
upnsSharedPointer<T> static_pointer_cast(upnsSharedPointer<U> const & r)
{
    return std::static_pointer_cast<T>(r);
}
#endif
template <typename T1, typename T2>
using upnsPair = std::pair<T1, T2>;
#endif

typedef upnsString CommitId;
typedef upnsuint64 MapIdentifier;
typedef upnsuint64 LayerIdentifier;
typedef upnsuint64 EntityIdentifier;

typedef upnsuint32 LockHandle;
typedef upnsuint32 StatusCode;

typedef upnsString CommitId;
typedef upnsString ObjectId;

// Path do not need to start with "/".
// Empty "directories" are not allowed ("//").
// Trailing "/" are omitted.
typedef upnsString Path;

#define InvalidCommitId "invalidCId"
#define InvalidObjectId "invalidOId"

typedef upnsPair<MapIdentifier, StatusCode> StatusPair;
typedef upnsVec<StatusPair> MapResultsVector;

class OperationDescription;
typedef upnsPair<StatusCode, OperationDescription> OperationResult;
}

#endif // __UPNS_TYPEDEFS_H
