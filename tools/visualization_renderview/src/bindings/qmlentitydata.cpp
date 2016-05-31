#include "bindings/qmlentitydata.h"

QmlEntitydata::QmlEntitydata()
    :m_entitydata( nullptr )
{

}

QmlEntitydata::QmlEntitydata(upns::upnsSharedPointer<upns::AbstractEntityData> &entitydata)
    :m_entitydata(entitydata)
{

}
