#include "upns/ui/bindings/qmlentitydatarenderer.h"
#include <upns/abstractentitydata.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <upns/layertypes/lastype.h>
#include <upns/layertypes/tflayer.h>
#include "qpointcloudgeometry.h"
#include "qpointcloud.h"
#include <QMatrix4x4>

QmlEntitydataRenderer::QmlEntitydataRenderer(Qt3DCore::QNode *parent)
    : QGeometryRenderer(parent),
      m_entitydata(NULL)
{
    QPointcloudGeometry *geometry = new QPointcloudGeometry(this);
    QGeometryRenderer::setGeometry(geometry);
    QGeometryRenderer::setPrimitiveType(QGeometryRenderer::Points); //TODO: To fix error where IndexAttribute is called and is null
}

QmlEntitydata *QmlEntitydataRenderer::entitydata() const
{
    return m_entitydata;
}

void QmlEntitydataRenderer::setEntitydata(QmlEntitydata *entitydata)
{
    if (m_entitydata != entitydata)
    {
        if(m_entitydata)
        {
            disconnect(m_entitydata, &QmlEntitydata::internalEntitydataChanged, this, &QmlEntitydataRenderer::setEntitydata);
        }
        m_entitydata = entitydata;
        if(m_entitydata)
        {
            connect(m_entitydata, &QmlEntitydata::internalEntitydataChanged, this, &QmlEntitydataRenderer::setEntitydata);
        }
    }
    updateGeometry();
    Q_EMIT entitydataChanged(entitydata);
}

void QmlEntitydataRenderer::updateGeometry()
{
    std::shared_ptr<upns::AbstractEntitydata> ed = m_entitydata->getEntitydata();

    if(geometry() != NULL)
    {
        geometry()->deleteLater();
    }

    if(ed == nullptr) return;

    if(strcmp(ed->type(), PointcloudEntitydata::TYPENAME()) == 0)
    {
        QGeometryRenderer::setPrimitiveType(QGeometryRenderer::Points);
        QGeometryRenderer::setGeometry(new QPointcloudGeometry(this));
        QPointcloud *pointcloud(new QPointcloud(this));
        pointcloud->setPointcloud(*std::static_pointer_cast< PointcloudEntitydata >(ed)->getData());
        QPointcloudGeometry *pointcloudGeometry = static_cast<QPointcloudGeometry *>(geometry());
        QMetaObject::invokeMethod(pointcloudGeometry, "setPointcloud", Qt::QueuedConnection, Q_ARG(QPointcloud *, pointcloud) );
        //pointcloudGeometry->setPointcloud(pointcloud);
    }
    else if(strcmp(ed->type(), LASEntitydata::TYPENAME()) == 0)
    {
        QGeometryRenderer::setPrimitiveType(QGeometryRenderer::Points);
        QGeometryRenderer::setGeometry(new QPointcloudGeometry(this));
        QPointcloud *pointcloud(new QPointcloud(this));
        std::unique_ptr<LASEntitydataReader> reader = std::static_pointer_cast< LASEntitydata >(ed)->getReader();
        pointcloud->read(reader->getReaderRaw());
        QPointcloudGeometry *pointcloudGeometry = static_cast<QPointcloudGeometry *>(geometry());
        QMetaObject::invokeMethod(pointcloudGeometry, "setPointcloud", Qt::QueuedConnection, Q_ARG(QPointcloud *, pointcloud) );
    }
//    else if(strcmp(ed->type(), OctomapEntitydata::TYPENAME()) == 0)
//    {
//    }
    else if(strcmp(ed->type(), TfEntitydata::TYPENAME()) == 0)
    {
        QMatrix4x4 mat( std::static_pointer_cast< TfEntitydata >(ed)->getData()->data() );
        qDebug() << mat;
    }
//    else if(strcmp(ed->type(), AssetEntitydata::TYPENAME()) == 0)
//    {
//    }
    else
    {
        qDebug() << "unknown entitytype for visualization";
    }

}
