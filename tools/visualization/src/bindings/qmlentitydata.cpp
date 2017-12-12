#include "upns/ui/bindings/qmlentitydata.h"
#include "entitydataloader.h"

QmlEntitydata::QmlEntitydata(QObject *parent)
    :QObject(parent),
     m_entitydata( NULL ),
     m_checkout( NULL ),
     m_path( "" ),
     m_isLoading( false ),
     m_edLoader(nullptr)
{

}

QmlEntitydata::QmlEntitydata(std::shared_ptr<upns::AbstractEntitydata> &entitydata, QmlCheckout *co, QString path)
    :m_entitydata(entitydata),
     m_checkout( co ),
     m_path( path ),
     m_isLoading( false ),
     m_edLoader(nullptr)
{
    if(m_checkout)
    {
        connect(m_checkout, &QmlCheckout::internalCheckoutChanged, this, &QmlEntitydata::setCheckout);
    }
}

QmlEntitydata::~QmlEntitydata()
{
    if(m_edLoader) delete m_edLoader;
}

std::shared_ptr<upns::AbstractEntitydata> QmlEntitydata::getEntitydata() { return m_entitydata; }

QString QmlEntitydata::path() const
{
    return m_path;
}

QmlCheckout *QmlEntitydata::checkout() const
{
    return m_checkout;
}

void QmlEntitydata::updateInfo()
{
    m_info = QJsonObject();
    Q_EMIT infoChanged(m_info);
    if(m_entitydata == nullptr) return;
    m_isLoading = true;
    Q_EMIT isLoadingChanged(m_isLoading);
    if(m_edLoader) delete m_edLoader;
    m_edLoader = new EntitydataLoader(this, m_checkout->getCheckoutObj(), m_path);
    connect(m_edLoader, &EntitydataLoader::entityInfoLoaded, this, &QmlEntitydata::setInfo);
    m_edLoader->start();
//    m_info = QJsonObject();
//    if(m_entitydata == nullptr) return;
////#if WITH_PCL OR/AND OPENVDB
//    if(strcmp(m_entitydata->type(), PointcloudEntitydata::TYPENAME()) == 0 || strcmp(m_entitydata->type(), FloatGridEntitydata::TYPENAME()) == 0)
//    {
////        if(   propertyName.compare("width") == 0
////           || propertyName.compare("length") == 0 )
////        {
//            std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>( m_entitydata );
//            if(entityData != nullptr)
//            {
//                upnsPointcloud2Ptr pc2 = entityData->getData();
//                m_info["width"] = QJsonValue(static_cast<qint64>(pc2->width));
//                m_info["length"] = QJsonValue(static_cast<qint64>(pc2->width));
//                //return QVariant(pc2->width);
//            }
////        }
////        if(   propertyName.compare("fields") == 0)
////        {
//            std::shared_ptr<PointcloudEntitydata> entitydataPointcloud;
//            std::shared_ptr<FloatGridEntitydata> entitydataOpenVDB;
//            if(entitydataPointcloud = std::dynamic_pointer_cast<PointcloudEntitydata>( m_entitydata ))
//            {
//                upnsPointcloud2Ptr pc2 = entitydataPointcloud->getData();
//                QJsonArray returnPointfieldList;
//                for(std::vector<::pcl::PCLPointField>::iterator iter(pc2->fields.begin());
//                    iter != pc2->fields.end();
//                    iter++)
//                {
//                    //TODO: this is only deleted, when entitydata is deleted via QObject parents.
//                    // Lifecycle is the same as m_entity! After m_entity changed, these pointfields must not be accessed anymore.
//                    QJsonObject field;
//                    field["name"] = QJsonValue( QString::fromStdString( iter->name ) );
//                    returnPointfieldList.append(field);
//                    //returnPointfieldList.append(QVariant::fromValue(new QPointfield(this, &(*iter))));
//                }
//                m_info["fields"] = returnPointfieldList;
//                //return returnPointfieldList;
//            }
//            else if(entitydataOpenVDB = std::dynamic_pointer_cast<FloatGridEntitydata>( m_entitydata ))
//            {
//                //For now ovdb is visualized as a pointcloud, so
//                // this emulates pointcloud visualization for float grids
//                QJsonArray returnPointfieldList;
//                QJsonObject jsX; jsX["name"] = "x";
//                QJsonObject jsY; jsY["name"] = "y";
//                QJsonObject jsZ; jsZ["name"] = "z";
//                QJsonObject jsNX; jsNX["name"] = "normal_x";
//                QJsonObject jsNY; jsNY["name"] = "normal_y";
//                QJsonObject jsNZ; jsNZ["name"] = "normal_z";
//                returnPointfieldList.append(jsX);
//                returnPointfieldList.append(jsY);
//                returnPointfieldList.append(jsZ);
//                returnPointfieldList.append(jsNX);
//                returnPointfieldList.append(jsNY);
//                returnPointfieldList.append(jsNZ);
//                m_info["fields"] = returnPointfieldList;
//                //return returnPointfieldList;
//            }
//            //return QVariant(0);
////        }
//    }
//    else
////#endif
//#if WITH_LAS
//    if(strcmp(m_entitydata->type(), LASEntitydata::TYPENAME()) == 0)
//    {
////        if(   propertyName.compare("width") == 0
////           || propertyName.compare("length") == 0 )
////        {
//            std::shared_ptr<LASEntitydata> entityData = std::dynamic_pointer_cast<LASEntitydata>( m_entitydata );
//            if(entityData != nullptr)
//            {
//                std::unique_ptr<LASEntitydataReader> rdr = entityData->getReader();
//                qint64 w = static_cast<qint64>(rdr->GetHeader().GetPointRecordsCount());
//                m_info["width"] = QJsonValue(w);
//                m_info["length"] = QJsonValue(w);
//            }
////        }
////        else if(   propertyName.compare("min") == 0)
////        {
//            //std::shared_ptr<LASEntitydata> entityData = std::dynamic_pointer_cast<LASEntitydata>( m_entitydata );
//            if(entityData == nullptr)
//            {
//                QJsonObject minVec3;
//                minVec3["x"] = QJsonValue(std::numeric_limits<float>::quiet_NaN());
//                minVec3["y"] = QJsonValue(std::numeric_limits<float>::quiet_NaN());
//                minVec3["z"] = QJsonValue(std::numeric_limits<float>::quiet_NaN());
//                m_info["min"] = minVec3;
//                //return QVariant(QVector3D(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()));
//            }
//            else
//            {
//                std::unique_ptr<LASEntitydataReader> rdr = entityData->getReader();
//                QJsonObject minVec3;
//                minVec3["x"] = QJsonValue(rdr->GetHeader().GetMinX());
//                minVec3["y"] = QJsonValue(rdr->GetHeader().GetMinY());
//                minVec3["z"] = QJsonValue(rdr->GetHeader().GetMinZ());
//                m_info["min"] = minVec3;
//                //return QVariant(QVector3D(rdr->GetHeader().GetMinX(),rdr->GetHeader().GetMinY(),rdr->GetHeader().GetMinZ()));
//            }
////        }
////        else if(   propertyName.compare("max") == 0)
////        {
//            //std::shared_ptr<LASEntitydata> entityData = std::dynamic_pointer_cast<LASEntitydata>( m_entitydata );
//            if(entityData == nullptr)
//            {
//                QJsonObject maxVec3;
//                maxVec3["x"] = QJsonValue(std::numeric_limits<float>::quiet_NaN());
//                maxVec3["y"] = QJsonValue(std::numeric_limits<float>::quiet_NaN());
//                maxVec3["z"] = QJsonValue(std::numeric_limits<float>::quiet_NaN());
//                m_info["min"] = maxVec3;
//                //return QVariant(QVector3D(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()));
//            }
//            else
//            {
//                std::unique_ptr<LASEntitydataReader> rdr = entityData->getReader();
//                QJsonObject maxVec3;
//                maxVec3["x"] = QJsonValue(rdr->GetHeader().GetMinX());
//                maxVec3["y"] = QJsonValue(rdr->GetHeader().GetMinY());
//                maxVec3["z"] = QJsonValue(rdr->GetHeader().GetMinZ());
//                m_info["min"] = maxVec3;
//                //return QVariant(QVector3D(rdr->GetHeader().GetMaxX(),rdr->GetHeader().GetMaxY(),rdr->GetHeader().GetMaxZ()));
//            }
////        }
////        else if(   propertyName.compare("fields") == 0)
////        {
//            //TODO: LAS has no "fields" nevertheless it has x, y, c and data properties. See Qt3DPontcloudRenderer.
//            QJsonArray returnPointfieldList;
//            QJsonObject jsX; jsX["name"] = "x";
//            QJsonObject jsY; jsY["name"] = "y";
//            QJsonObject jsZ; jsZ["name"] = "z";
//            returnPointfieldList.append(jsX);
//            returnPointfieldList.append(jsY);
//            returnPointfieldList.append(jsZ);
//            m_info["fields"] = returnPointfieldList;
//        //}
//    }
//    else
//#endif // WITH_LAS
////    if(strcmp(ed->type(), OctomapEntitydata::TYPENAME()) == 0)
////    {
////    }
//    if(strcmp(m_entitydata->type(), TfEntitydata::TYPENAME()) == 0)
//    {
//        // has special qml wrapper
//    }
////    else if(strcmp(m_entitydata->type(), AssetEntitydata::TYPENAME()) == 0)
////    {
////    }
//    else if(strcmp(m_entitydata->type(), PosePathEntitydata::TYPENAME()) == 0)
//    {
////        if(   propertyName.compare("width") == 0
////           || propertyName.compare("length") == 0 )
////        {
//            std::shared_ptr<PosePathEntitydata> entityData = std::dynamic_pointer_cast<PosePathEntitydata>( m_entitydata );
//            if(entityData != nullptr)
//            {
//            PosePathPtr pp = entityData->getData();
////            return QVariant(pp->poses_size());
//            qint64 w = static_cast<qint64>(pp->poses_size());
//            m_info["width"] = QJsonValue(w);
//            m_info["length"] = QJsonValue(w);
//            }
////        }
//    }
//    else
//    {
//        qDebug() << "unknown entitytype for visualization";
//    }
//    Q_EMIT infoChanged(m_info);
}

QJsonObject QmlEntitydata::info() const
{
    return m_info;
}

bool QmlEntitydata::isLoading() const
{
    return m_isLoading;
}

void QmlEntitydata::setCheckout(QmlCheckout *checkout)
{
    bool changed = false;
    if (m_checkout != checkout)
    {
        if(m_checkout)
        {
            disconnect(m_checkout, &QmlCheckout::internalCheckoutChanged, this, &QmlEntitydata::setCheckout);
        }
        m_checkout = checkout;
        if(m_checkout)
        {
            connect(m_checkout, &QmlCheckout::internalCheckoutChanged, this, &QmlEntitydata::setCheckout);
        }
        Q_EMIT checkoutChanged(checkout);
        changed = true;
    }
    if( !m_path.isEmpty() && m_checkout->getCheckoutObj() )
    {
        m_entitydata = m_checkout->getCheckoutObj()->getEntitydataReadOnly(m_path.toStdString());
        Q_EMIT internalEntitydataChanged( this );

        changed = true;
    }
    if(changed)
    {
        //Note: this is wrong sequence. updateInfo should be called before internalEntitydataChanged and checkoutChanged (?).
        updateInfo();
        Q_EMIT updated();
    }
}

void QmlEntitydata::setPath(QString path)
{
    if (m_path == path)
        return;

    m_path = path;
    if( m_checkout && m_checkout->getCheckoutObj() && !m_path.isEmpty() )
    {
        std::shared_ptr<mapit::msgs::Entity> e = m_checkout->getCheckoutObj()->getEntity(m_path.toStdString());
        if(e)
        {
            m_entitydata = m_checkout->getCheckoutObj()->getEntitydataReadOnly(m_path.toStdString());
            Q_EMIT internalEntitydataChanged( this );
        }
    }
    updateInfo();
    Q_EMIT pathChanged(path);
    Q_EMIT updated();
}

void QmlEntitydata::setInfo(QJsonObject info)
{
    m_isLoading = false;
    m_info = info;
    Q_EMIT isLoadingChanged(m_isLoading);
    Q_EMIT infoChanged(info);
}
