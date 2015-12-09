#include "qmllayer.h"

QmlLayer::QmlLayer(upns::Layer *obj)
    :m_layer(obj)
{
    for(int i=0 ; m_layer->entities_size() > i ; ++i)
    {
        m_entities.append(new QmlEntity(m_layer->mutable_entities(i)));
    }
}

QmlLayer::~QmlLayer()
{
    QList<QmlEntity*>::iterator iter(m_entities.begin());
    while( iter != m_entities.end())
    {
        delete *iter;
        iter++;
    }
}

QmlEntity *QmlLayer::addEntity()
{
    upns::Entity* e = this->m_layer->add_entities();
    m_entities.append( new QmlEntity(e) );
    return m_entities.back();
}

//void QmlLayer::entities_append(QQmlListProperty<QmlEntity> *property, QmlEntity *value)
//{
//    QmlLayer *that = static_cast<QmlLayer*>( property->object );
//}

int QmlLayer::entities_count(QQmlListProperty<QmlEntity> *property)
{
    QmlLayer *that = static_cast<QmlLayer*>( property->object );
    return that->m_layer->entities_size();
}

QmlEntity *QmlLayer::entities_at(QQmlListProperty<QmlEntity> *property, int index)
{
    QmlLayer *that = static_cast<QmlLayer*>( property->object );
    return that->m_entities.at(index);
}

//void QmlLayer::entities_clear(QQmlListProperty<QmlEntity> *property)
//{

//}
