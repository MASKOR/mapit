/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2015 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef __VISUAL_GLOBALS_H
#define __VISUAL_GLOBALS_H



namespace mapit {

#ifdef USE_QT_STRUCTURES
#define upnsToQVector(vec) (vec)
#else // USE_QT_STRUCTURES

#ifdef QT_CORE_LIB
#include <QVector>
}

// Not in namespace mapit so this can be replaced by a macro when qt types are used.
template<class T>
QVector<T> upnsToQVector(std::vector<T> vec)
{
    return QVector<T>::fromStdVector(vec);
}

namespace mapit {
#endif // QT_CORE_LIB

#ifdef USE_BOOST_STRUCTURES

#else // USE_BOOST_STRUCTURES

#endif // USE_BOOST_STRUCTURES

#endif // USE_QT_STRUCTURES

}

#endif
