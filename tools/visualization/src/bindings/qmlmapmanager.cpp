#include "qmlmapmanager.h"
#include "visualization_globals.h"
#include <QVector>

QmlMapManager::QmlMapManager()
    :m_mapManager( NULL )
{

}

QmlMapManager::~QmlMapManager()
{
    if(m_mapManager)
    {
        delete m_mapManager;
    }
}

QList<QString> QmlMapManager::listMaps()
{
    if(!m_mapManager) initialize();
    QList<QString> ret;
    Q_FOREACH( const quint64 &ui64, m_mapManager->listMaps())
    {
        ret.push_back( QString::number( ui64 ) );
    }
    return ret;//QList<qint32>::fromVector( upnsToQVector() );
}

QmlMap *QmlMapManager::getMap(quint64 mapId)
{
    if(!m_mapManager) initialize();
    upns::upnsSharedPointer<upns::Map> map = m_mapManager->getMap(mapId);
    return new QmlMap(map); // Qml takes ownership
}

bool QmlMapManager::canRead()
{
    if(!m_mapManager) initialize();
    return m_mapManager->canRead();
}

bool QmlMapManager::canWrite()
{
    if(!m_mapManager) initialize();
    return m_mapManager->canWrite();
}

QJsonObject QmlMapManager::doOperation(const QJsonObject &desc)
{
    upns::OperationDescription od;
    operationDescriptionFromJsonObject( od, desc);
    QJsonObject json;
    upns::OperationResult result = m_mapManager->doOperation( od );
    json["status"] = static_cast<int>(result.first); //< zero remains zero, can be compared to other reinterpret casted values.
    json["output"] = operationDescriptionToJson( result.second );
    return json;
}

upns::MapManager *QmlMapManager::getMapManager()
{
    if(!m_mapManager) initialize();
    return m_mapManager;
}

void QmlMapManager::initialize()
{
    //TODO: parse qml -> yaml
    YAML::Node conf;
    YAML::Node mapsource;
    mapsource["name"] = "MapFileService";
    mapsource["filename"] = "test2.db";
    conf["mapsource"] = mapsource;
    m_mapManager = new upns::MapManager(conf);
}

QJsonObject QmlMapManager::operationDescriptionToJson(const upns::OperationDescription &desc) const
{
    QJsonObject jOperDesc;
    jOperDesc["operatorname"] = QString::fromStdString(desc.operatorname());
    jOperDesc["operatorversion"] = desc.operatorversion();
    //QJsonArray jParams;
    QJsonObject jParams;
    for(int i=0; i < desc.params_size() ; ++i)
    {
        QJsonObject jp;
        const upns::OperationParameter *p = &desc.params(i);
        //jp["key"] = p->key();
        if(!p->strval().empty())
        {
            jp["strval"] = QString::fromStdString(p->strval());
        }
        if(p->intval() != 0)
        {
            jp["intval"] = p->intval();
        }
        if(p->realval() != 0.0)
        {
            jp["realval"] = p->realval();
        }
        if(p->entityval() != 0)
        {
            jp["entityval"] = QString::number(p->entityval());
        }
        if(p->layerval() != 0)
        {
            jp["layerval"] = QString::number(p->layerval());
        }
        if(p->mapval() != 0)
        {
            jp["mapval"] = QString::number(p->mapval());
        }
        //jParams.append(jp);
        jParams[QString::fromStdString(p->key())] = jp;
    }
    jOperDesc["params"] = jParams;
    return jOperDesc;
}

void QmlMapManager::yamlFromJsonObject(YAML::Node &yaml, const QJsonObject &json) const
{
    QJsonObject::const_iterator iter(json.constBegin());
    while(iter != json.constEnd())
    {
        if(iter.value().isObject())
        {
            YAML::Node childNode;
            yamlFromJsonObject(childNode, iter.value().toObject());
            yaml[iter.key().toStdString()] = childNode;
        }
        else if(iter.value().isArray())
        {
            log_error("array json could not be converted to yaml (not yet implemented)");
//            YAML::Node childNode;
//            yamlFromJsonObject(childNode, iter.value().toArray());
//            yaml[iter.key()] = childNode;
        }
        else if(iter.value().isString())
        {
            yaml[iter.key().toStdString()] = iter.value().toString().toStdString();
        }
        else if(iter.value().isBool())
        {
            yaml[iter.key().toStdString()] = iter.value().toBool();
        }
        else if(iter.value().isDouble())
        {
            yaml[iter.key().toStdString()] = iter.value().toDouble();
        }
        else
        {
            log_error("unknown json could not be converted to yaml");
        }
        iter++;
    }
}

void QmlMapManager::operationDescriptionFromJsonObject(upns::OperationDescription &opDesc, const QJsonObject &json) const
{
    QJsonObject::const_iterator iter(json.constBegin());
    while(iter != json.constEnd())
    {
        if(iter.key() == "operatorname")
        {
            opDesc.set_operatorname(iter.value().toString().toStdString());
        }
        else if(iter.key() == "operatorversion")
        {
            opDesc.set_operatorversion(iter.value().toInt());
        } else if(iter.key() == "params")
        {
            if(!iter.value().isArray())
            {
                log_error("json operator description malformed, has no list of params");
                continue;
            }
            const QJsonArray &params = iter.value().toArray();
            QJsonArray::const_iterator paramIter(params.constBegin());
            while(paramIter != params.constEnd())
            {
                upns::OperationParameter* param = opDesc.add_params();
                const QJsonObject &jparam = paramIter->toObject();
                if(!jparam.contains("key") || !jparam["key"].isString())
                {
                    log_error("json operator description malformed, has no key");
                    continue;
                }
                param->set_key(jparam["key"].toString().toStdString());
                if(jparam.contains("strval"))
                {
                    param->set_strval( jparam["strval"].toString().toStdString() );
                }
                if(jparam.contains("intval"))
                {
                    const QJsonValue val = jparam["intval"];
                    if( val.isDouble() )
                    {
                        param->set_intval( val.toInt() );
                    }
                    else if( val.isString() )
                    {
                        param->set_intval( val.toString().toInt() );
                    }
                    else if( val.isBool() )
                    {
                        param->set_intval( val.toBool()?1:0 );
                    }
                    else Q_ASSERT( false );
                }
                if(jparam.contains("realval"))
                {
                    const QJsonValue val = jparam["realval"];
                    if( val.isDouble() )
                    {
                        param->set_realval( val.toDouble() );
                    }
                    else if( val.isString() )
                    {
                        param->set_realval( val.toString().toDouble() );
                    }
                    else if( val.isBool() )
                    {
                        param->set_realval( val.toBool()?1.0:0.0 );
                    }
                    else Q_ASSERT( false );
                }
                if(jparam.contains("entityval"))
                {
                    param->set_entityval( jparam["entityval"].toString().toULongLong() );
                }
                if(jparam.contains("layerval"))
                {
                    param->set_layerval( jparam["layerval"].toString().toULongLong() );
                }
                if(jparam.contains("mapval"))
                {
                    param->set_mapval( jparam["mapval"].toString().toULongLong() );
                }
                if(jparam.contains("transformval"))
                {
                    log_error("not yet implemented to set transfrom val");
                }
//                else
//                {
//                    log_error("unknown json could not be converted to operation description."
//                              "Check for lower case 'V' and correct member."
//                              "Key: ’"+ jparam["key"].toString().toStdString() + "’");
//                }
                paramIter++;
            }
        }
        else
        {
            log_error("json operator description malformed");
        }
        iter++;
    }
}
