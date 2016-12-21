#include "qmlentity.h"
#include "upns.h"
#include "libs/upns_interface/services.pb.h"

QmlEntity::QmlEntity(QObject *parent)
    :QObject(parent),m_entity( nullptr )
{

}

QmlEntity::QmlEntity(upns::upnsSharedPointer<upns::Entity> &obj)
    :m_entity( obj )
{

}


bool QmlEntity::isValid() const
{
    return m_entity != nullptr;
}
