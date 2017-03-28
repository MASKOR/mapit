#ifndef __VISUAL_GLOBALS_H
#define __VISUAL_GLOBALS_H



namespace upns {

#ifdef USE_QT_STRUCTURES
#define upnsToQVector(vec) (vec)
#else // USE_QT_STRUCTURES

#ifdef QT_CORE_LIB
#include <QVector>
}

// Not in namespace upns so this can be replaced by a macro when qt types are used.
template<class T>
QVector<T> upnsToQVector(std::vector<T> vec)
{
    return QVector<T>::fromStdVector(vec);
}

namespace upns {
#endif // QT_CORE_LIB

#ifdef USE_BOOST_STRUCTURES

#else // USE_BOOST_STRUCTURES

#endif // USE_BOOST_STRUCTURES

#endif // USE_QT_STRUCTURES

}

#endif
