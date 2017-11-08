#include "upns/ui/bindings/qmlentitydata.h"
#include <upns/layertypes/pointcloudlayer.h>
#if WITH_LAS
#include <upns/layertypes/lastype.h>
#endif // WITH_LAS
#include <upns/layertypes/openvdblayer.h>
#include <upns/layertypes/tflayer.h>
#include <upns/layertypes/pose_path.h>
#include <QVector3D>
#include <qpointfield.h>

QmlEntitydata::QmlEntitydata(QObject *parent)
    :QObject(parent),
     m_entitydata( NULL ),
     m_checkout( NULL ),
     m_path( "" )
{

}

QmlEntitydata::QmlEntitydata(std::shared_ptr<upns::AbstractEntitydata> &entitydata, QmlCheckout *co, QString path)
    :m_entitydata(entitydata),
     m_checkout( co ),
     m_path( path )
{
    if(m_checkout)
    {
        connect(m_checkout, &QmlCheckout::internalCheckoutChanged, this, &QmlEntitydata::setCheckout);
    }
}

QString QmlEntitydata::path() const
{
    return m_path;
}

QmlCheckout *QmlEntitydata::checkout() const
{
    return m_checkout;
}

QVariant QmlEntitydata::getInfo(QString propertyName)
{
    if(m_entitydata == nullptr) return QVariant(0);
//#if WITH_PCL OR/AND OPENVDB
    if(strcmp(m_entitydata->type(), PointcloudEntitydata::TYPENAME()) == 0 || strcmp(m_entitydata->type(), FloatGridEntitydata::TYPENAME()) == 0)
    {
        if(   propertyName.compare("width") == 0
           || propertyName.compare("length") == 0 )
        {
            std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>( m_entitydata );
            if(entityData == nullptr)
            {
                return QVariant(0);
            }
            upnsPointcloud2Ptr pc2 = entityData->getData();
            return QVariant(pc2->width);
        }
        if(   propertyName.compare("fields") == 0)
        {
            std::shared_ptr<PointcloudEntitydata> entitydataPointcloud;
            std::shared_ptr<FloatGridEntitydata> entitydataOpenVDB;
            if(entitydataPointcloud = std::dynamic_pointer_cast<PointcloudEntitydata>( m_entitydata ))
            {
                upnsPointcloud2Ptr pc2 = entitydataPointcloud->getData();
                QVariantList returnPointfieldList;
                for(std::vector<::pcl::PCLPointField>::iterator iter(pc2->fields.begin());
                    iter != pc2->fields.end();
                    iter++)
                {
                    //TODO: this is only deleted, when entitydata is deleted via QObject parents.
                    // Lifecycle is the same as m_entity! After m_entity changed, these pointfields must not be accessed anymore.
                    returnPointfieldList.append(QVariant::fromValue(new QPointfield(this, &(*iter))));
                }
                return returnPointfieldList;
            }
            else if(entitydataOpenVDB = std::dynamic_pointer_cast<FloatGridEntitydata>( m_entitydata ))
            {
                //For now ovdb is visualized as a pointcloud, so
                // this emulates pointcloud visualization for float grids
                QVariantList returnPointfieldList;
                QJsonObject jsX; jsX["name"] = "x";
                QJsonObject jsY; jsY["name"] = "y";
                QJsonObject jsZ; jsZ["name"] = "z";
                QJsonObject jsNX; jsNX["name"] = "normal_x";
                QJsonObject jsNY; jsNY["name"] = "normal_y";
                QJsonObject jsNZ; jsNZ["name"] = "normal_z";
                returnPointfieldList.append(QVariant::fromValue(jsX));
                returnPointfieldList.append(QVariant::fromValue(jsY));
                returnPointfieldList.append(QVariant::fromValue(jsZ));
                returnPointfieldList.append(QVariant::fromValue(jsNX));
                returnPointfieldList.append(QVariant::fromValue(jsNY));
                returnPointfieldList.append(QVariant::fromValue(jsNZ));
                return returnPointfieldList;
            }
            return QVariant(0);
        }
    }
    else
//#endif
#if WITH_LAS
    if(strcmp(m_entitydata->type(), LASEntitydata::TYPENAME()) == 0)
    {
        if(   propertyName.compare("width") == 0
           || propertyName.compare("length") == 0 )
        {
            std::shared_ptr<LASEntitydata> entityData = std::dynamic_pointer_cast<LASEntitydata>( m_entitydata );
            if(entityData == nullptr)
            {
                return QVariant(0);
            }
            std::unique_ptr<LASEntitydataReader> rdr = entityData->getReader();
            return QVariant(rdr->GetHeader().GetPointRecordsCount());
        }
        else if(   propertyName.compare("min") == 0)
        {
            std::shared_ptr<LASEntitydata> entityData = std::dynamic_pointer_cast<LASEntitydata>( m_entitydata );
            if(entityData == nullptr)
            {
                return QVariant(QVector3D(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()));
            }
            std::unique_ptr<LASEntitydataReader> rdr = entityData->getReader();
            return QVariant(QVector3D(rdr->GetHeader().GetMinX(),rdr->GetHeader().GetMinY(),rdr->GetHeader().GetMinZ()));
        }
        else if(   propertyName.compare("max") == 0)
        {
            std::shared_ptr<LASEntitydata> entityData = std::dynamic_pointer_cast<LASEntitydata>( m_entitydata );
            if(entityData == nullptr)
            {
                return QVariant(QVector3D(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()));
            }
            std::unique_ptr<LASEntitydataReader> rdr = entityData->getReader();
            return QVariant(QVector3D(rdr->GetHeader().GetMaxX(),rdr->GetHeader().GetMaxY(),rdr->GetHeader().GetMaxZ()));
        }
        else if(   propertyName.compare("fields") == 0)
        {
            //TODO: LAS has no "fields" nevertheless it has x, y, c and data properties. See Qt3DPontcloudRenderer.
        }
    }
    else
#endif // WITH_LAS
//    if(strcmp(ed->type(), OctomapEntitydata::TYPENAME()) == 0)
//    {
//    }
    if(strcmp(m_entitydata->type(), TfEntitydata::TYPENAME()) == 0)
    {
        // has special qml wrapper
    }
//    else if(strcmp(m_entitydata->type(), AssetEntitydata::TYPENAME()) == 0)
//    {
//    }
    else if(strcmp(m_entitydata->type(), PosePathEntitydata::TYPENAME()) == 0)
    {
        if(   propertyName.compare("width") == 0
           || propertyName.compare("length") == 0 )
        {
            std::shared_ptr<PosePathEntitydata> entityData = std::dynamic_pointer_cast<PosePathEntitydata>( m_entitydata );
            if(entityData == nullptr)
            {
                return QVariant(0);
            }
            PosePathPtr pp = entityData->getData();
            return QVariant(pp->poses_size());
        }
    }
    else
    {
        qDebug() << "unknown entitytype for visualization";
    }
    return QVariant();
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
    Q_EMIT pathChanged(path);
    Q_EMIT updated();
}
