#ifndef ENTITYSTREAMMANAGER_H
#define ENTITYSTREAMMANAGER_H

#include "upns_globals.h"
#include "abstractentitydata.h"
#include "modules/serialization/abstractmapserializerNEW.h" //< TODO: move. abstract serializer not needed by modules

namespace upns {

class EntityStreamManager
{
public:
    static upnsSharedPointer<AbstractEntityData> getEntityDataImpl(AbstractMapSerializer *serializer, const ObjectId &entityId, bool canRead, bool canWrite);
};

}
#endif
