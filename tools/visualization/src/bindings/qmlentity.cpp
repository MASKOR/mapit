#include "upns/ui/bindings/qmlentity.h"
#include <upns/typedefs.h>
#include <upns/services.pb.h>

QmlEntity::QmlEntity(QObject *parent)
    :QObject(parent),m_entity( nullptr )
{

}

QmlEntity::QmlEntity(std::shared_ptr<upns::Entity> &obj)
    :m_entity( obj )
{

}


bool QmlEntity::isValid() const
{
    return m_entity != nullptr;
}
