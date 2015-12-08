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

qint32 QmlMapManager::doOperation(const QJsonObject &desc)
{
    upns::OperationDescription od;
    operationDescriptionFromJsonObject( od, desc);
    m_mapManager->doOperation( od );
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
    mapsource["filename"] = "test.db";
    conf["mapsource"] = mapsource;
    m_mapManager = new upns::MapManager(conf);
}

void QmlMapManager::yamlFromJsonObject(YAML::Node &yaml, const QJsonObject &json)
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
            qDebug() << "array json could not be converted to yaml (not yet implemented)";
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
            qDebug() << "unknown json could not be converted to yaml";
        }
        iter++;
    }
}

void QmlMapManager::operationDescriptionFromJsonObject(upns::OperationDescription &opDesc, const QJsonObject &json)
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
                qDebug() << "json operator description malformed, has no list of params";
                continue;
            }
            const QJsonArray &params = iter.value().toArray();
            upns::OperationParameter* param = opDesc.add_params();
            QJsonArray::const_iterator paramIter(params.constBegin());
            while(paramIter != params.constEnd())
            {
                const QJsonObject &jparam = paramIter->toObject();
                if(!jparam.contains("key") || !jparam["key"].isString())
                {
                    qDebug() << "json operator description malformed, has no key";
                    continue;
                }
                param->set_key(jparam["key"].toString().toStdString());
                if(jparam.contains("strval"))
                {
                    param->set_strval( jparam["strval"].toString().toStdString() );
                }
                else if(jparam.contains("intval"))
                {
                    param->set_intval( jparam["intval"].toInt() );
                }
                else if(jparam.contains("realval"))
                {
                    param->set_realval( jparam["realval"].toDouble() );
                }
                else if(jparam.contains("entityVal"))
                {
                    param->mutable_entityval()->set_entityid( jparam["entityVal"].toInt() );
                }
                else if(jparam.contains("transformVal"))
                {
                    qDebug() << "not yet implemented to set transfrom val";
                }
                else
                {
                    qDebug() << "unknown json could not be converted to yaml";
                }
                paramIter++;
            }
        }
        else
        {
            qDebug() << "json operator description malformed";
        }
        iter++;
    }
}
