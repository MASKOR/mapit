#include "qmlmap.h"

QmlMap::QmlMap(upns::Map *obj)
    :m_map( obj )
{
    for(int i=0 ; m_map->layers_size() > i ; ++i)
    {
        m_layers.append(new QmlLayer(m_map->mutable_layers(i)));
    }
}

QmlMap::~QmlMap()
{
    QList<QmlLayer*>::iterator iter(m_layers.begin());
    while( iter != m_layers.end())
    {
        delete *iter;
        iter++;
    }
}

QmlLayer *QmlMap::addLayer()
{
    upns::Layer* l = m_map->add_layers();
    m_layers.append( new QmlLayer(l) );
    return m_layers.back();
}

int QmlMap::layers_count(QQmlListProperty<QmlLayer> *property)
{
    QmlMap *that = static_cast<QmlMap*>( property->object );
    return that->m_map->layers_size();
}

QmlLayer *QmlMap::layers_at(QQmlListProperty<QmlLayer> *property, int index)
{
    QmlMap *that = static_cast<QmlMap*>( property->object );
    return that->m_layers.at(index);
}
