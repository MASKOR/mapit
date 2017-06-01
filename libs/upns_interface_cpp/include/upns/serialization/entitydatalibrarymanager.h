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
