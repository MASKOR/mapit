#ifndef ENTITYSTREAMMANAGER_H
#define ENTITYSTREAMMANAGER_H

#include "upns_globals.h"
#include "abstractentitydata.h"
#include "abstractmapserializer.h"

/**
 *  Note: This class should usually not be used. It is located in the "include"
 *  folder, so it can be used for stubs and testing only (e.g. read pointclouds
 *  from file instead of version database).
 *  This class provides a loading mechanism for concrete layertype implementations.
 *  Given an abstract Serializer, the one Method of this class creates an entity
 *  which can be read or can be written to the stream. Internally this features
 *  loading of dynamically linked libraries. Thus there is no compiler dependency
 *  to all kinds of layertypes (e.g. PCL, OVDB, Octomap, ...).
 */

namespace upns {
class AbstractMapSerializer;
class EntityStreamManager
{
public:
    static upnsSharedPointer<AbstractEntityData> getEntityDataImpl(AbstractMapSerializer *serializer, const ObjectId &entityId, bool canRead/*, bool canWrite*/);
    static upnsSharedPointer<AbstractEntityData> getEntityDataFromPathImpl(AbstractMapSerializer *serializer, const Path &path, bool canRead, bool canWrite);
};

}
#endif
