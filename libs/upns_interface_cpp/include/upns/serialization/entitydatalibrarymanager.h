/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#ifndef ENTITYDATALIBRARYMANAGER_H
#define ENTITYDATALIBRARYMANAGER_H

#include <upns/typedefs.h>
#include <upns/abstractentitydata.h>

/**
 *  Note: This class should usually not be used directly. An implementation for
 *  this class is part of core. An own implementation may be needed. This
 *  class provides a loading mechanism for concrete layertype implementations. Given
 *  an abstract serializer, the one Method of this class creates an entity which can
 *  be read from or can be written to the stream. Internally this features loading
 *  of dynamically linked libraries. Thus there is no compiler dependency to all
 *  kinds of layertypes (e.g. PCL, OVDB, Octomap, ...).
 *
 *  E.g. An application/user requests a pointcloud. The Pointcloud is capsulated
 *  in a concrete "PointcloudEntitydata" Object. EntityDatalibraryManager has to create
 *  a "PointcloudEntity" internally from the data in the "streamprovider". The
 *  Pointcloud is then returned to the application as a AbstractEntitydata.
 *  AbstractEntitydata can the be casted to "PointcloudEntity" from the application.
 */

namespace upns {
class EntityDataLibraryManager
{
public:
    static std::shared_ptr<AbstractEntitydata> getEntitydataFromProvider(const std::string &type, std::shared_ptr<AbstractEntitydataProvider> streamprovider, bool canRead = true);
};

}

#endif
