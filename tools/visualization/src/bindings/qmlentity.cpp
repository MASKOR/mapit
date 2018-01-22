#include "upns/ui/bindings/qmlentity.h"
#include <upns/typedefs.h>
#include <mapit/msgs/services.pb.h>

QmlEntity::QmlEntity(QObject *parent)
    : QObject(parent)
    , m_entity( nullptr )
    , m_stamp( nullptr )
{

}

QmlEntity::QmlEntity(std::shared_ptr<mapit::msgs::Entity> &obj)
    : m_entity( obj )
    , m_stamp( nullptr )
{

}


bool QmlEntity::isValid() const
{
    return m_entity != nullptr;
}

QString QmlEntity::type() const
{
    if(!m_entity) return "";
    return QString::fromStdString(m_entity->type());
}

QString QmlEntity::frameId() const
{
    if(!m_entity) return "";
    return QString::fromStdString(m_entity->frame_id());
}

QmlStamp *QmlEntity::stamp()
{
    if(!m_entity) return nullptr;
    if(!m_stamp)
    {
        m_stamp = new QmlStamp(m_entity->mutable_stamp(), this);
    }
    return m_stamp;
}
