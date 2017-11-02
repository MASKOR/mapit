#include "upns/ui/bindings/qmlentity.h"
#include <upns/typedefs.h>
#include <mapit/msgs/services.pb.h>

QmlEntity::QmlEntity(QObject *parent)
    :QObject(parent),m_entity( nullptr )
{

}

QmlEntity::QmlEntity(std::shared_ptr<mapit::msgs::Entity> &obj)
    :m_entity( obj )
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

QString QmlEntity::stamp() const
{
    return QString::number(m_entity->stamp().sec()) + "s, " + QString::number(m_entity->stamp().nsec()) + "ns";
}
