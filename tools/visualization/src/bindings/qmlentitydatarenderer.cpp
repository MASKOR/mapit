#include "upns/ui/bindings/qmlentitydatarenderer.h"
#include <upns/abstractentitydata.h>
#include <upns/layertypes/pointcloudlayer.h>
#if WITH_LAS
#include <upns/layertypes/lastype.h>
#endif // WITH_LAS
//#if WITH_OPENVDB
#include <upns/layertypes/openvdblayer.h>
#include <qopenvdbgridpointsurfacegeometry.h>
//#endif // WITH_OPENVDB
#include <upns/layertypes/tflayer.h>
#include <upns/layertypes/pose_path.h>
#include <upns/layertypes/assettype.h>
#include "qpointcloudgeometry.h"
#include "qpointcloud.h"
#include "upns/ui/bindings/qmlpathgeometry.h"
#include "upns/ui/bindings/qmlplymeshgeometry.h"
#include <upns/logging.h>
#include <QMatrix4x4>

QmlEntitydataRenderer::QmlEntitydataRenderer(Qt3DCore::QNode *parent)
    : QGeometryRenderer(parent)
    , m_entitydata(nullptr)
    , m_coordinateSystem(nullptr)
{
    qRegisterMetaType<PosePathPtr>("PosePathPtr");
    qRegisterMetaType<AssetPtr>("AssetPtr");
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

void QmlEntitydataRenderer::setCoordinateSystem(QmlPointcloudCoordinatesystem *coordinateSystem)
{
    if (m_coordinateSystem == coordinateSystem)
        return;

    m_coordinateSystem = coordinateSystem;
    Q_EMIT coordinateSystemChanged(coordinateSystem);
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
        std::shared_ptr<PointcloudEntitydata> pcEd = std::dynamic_pointer_cast< PointcloudEntitydata >(ed);
        if(pcEd == nullptr)
        {
            qWarning() << "FATAL: Corrupt entitydata. Wrong type (not Pointcloud Entitydata)";
            return;
        }
        pointcloud->setPointcloud(*pcEd->getData());
        QPointcloudGeometry *pointcloudGeometry = static_cast<QPointcloudGeometry *>(geometry());
        QMetaObject::invokeMethod(pointcloudGeometry, "setPointcloud", Qt::QueuedConnection, Q_ARG(QPointcloud *, pointcloud) );
        //pointcloudGeometry->setPointcloud(pointcloud);
    }
#if WITH_LAS
    else if(strcmp(ed->type(), LASEntitydata::TYPENAME()) == 0)
    {
        QGeometryRenderer::setPrimitiveType(QGeometryRenderer::Points);
        QGeometryRenderer::setGeometry(new QPointcloudGeometry(this));
        QPointcloud *pointcloud(new QPointcloud(this));
        std::shared_ptr<LASEntitydata> lasEd = std::dynamic_pointer_cast< LASEntitydata >(ed);
        if(lasEd == nullptr)
        {
            qWarning() << "FATAL: Corrupt entitydata. Wrong type (not LAS Entitydata)";
            return;
        }
        std::unique_ptr<LASEntitydataReader> reader = lasEd->getReader();
        const int LEVEL_OF_DETAIL_TEMP=5; //TODO: skip 4 of 5 points for MILAN Dataset
        if(m_coordinateSystem)
        {
            pointcloud->read(reader->getReaderRaw()
                           , m_coordinateSystem->m_initialized
                           , &m_coordinateSystem->m_offsetX
                           , &m_coordinateSystem->m_offsetY
                           , &m_coordinateSystem->m_offsetZ
                           , false
                           , false
                           , 10.0f
                           , false
                           , LEVEL_OF_DETAIL_TEMP);
            QPointcloudGeometry *pointcloudGeometry = static_cast<QPointcloudGeometry *>(geometry());
            //TODO: A timer here should solve several crashes!
            //QMetaObject::Connection *conn = QMetaObject::connect(pointcloud, &QPointcloud::offsetChanged, [&conn](){});
            QMetaObject::invokeMethod(pointcloudGeometry, "setPointcloud", Qt::QueuedConnection, Q_ARG(QPointcloud *, pointcloud) );
            m_coordinateSystem->m_initialized = true; //TODO: Thread safe!
        }
        else
        {
            log_error("Pointcloud has no coordinate system! Can not be loaded.");
        }
    }
#endif // WITH_LAS
//    else if(strcmp(ed->type(), OctomapEntitydata::TYPENAME()) == 0)
//    {
//    }
    else if(strcmp(ed->type(), TfEntitydata::TYPENAME()) == 0)
    {
        std::shared_ptr<TfEntitydata> tfEd = std::dynamic_pointer_cast< TfEntitydata >(ed);
        if(tfEd == nullptr)
        {
            qWarning() << "FATAL: Corrupt entitydata. Wrong type (not a tf)";
            return;
        }
        Eigen::Affine3f tfMat = Eigen::Affine3f(
                                    tfEd->getData()->translation
                                  * tfEd->getData()->rotation
                                );
        QMatrix4x4 mat( &(tfMat.matrix()(0)) );
        qDebug() << mat;
    }
    else if(strcmp(ed->type(), AssetEntitydata::TYPENAME()) == 0)
    {
        QGeometryRenderer::setPrimitiveType(QGeometryRenderer::Triangles);
        QGeometryRenderer::setGeometry(new QmlPlyMeshGeometry(this));
        QGeometryRenderer::setIndexOffset(0);
        QGeometryRenderer::setFirstInstance(0);
        QGeometryRenderer::setInstanceCount(1);
        AssetPtr asset = std::dynamic_pointer_cast< AssetEntitydata >(ed)->getData();
        if(asset == nullptr)
        {
            qWarning() << "FATAL: Corrupt entitydata. Wrong type (not an asset)";
            return;
        }
        QmlPlyMeshGeometry *geom = static_cast<QmlPlyMeshGeometry *>(geometry());
        // TODO: does shared pointer survive here? Will the pointer be cleaned?
        QMetaObject::invokeMethod(geom, "setAsset", Qt::QueuedConnection, Q_ARG(AssetPtr, asset) );
    }
    else if(strcmp(ed->type(), PosePathEntitydata::TYPENAME()) == 0)
    {
        QGeometryRenderer::setPrimitiveType(QGeometryRenderer::LineStrip);
        QGeometryRenderer::setGeometry(new QmlPathGeometry(this));
        PosePathPtr path = std::dynamic_pointer_cast< PosePathEntitydata >(ed)->getData();
        if(path == nullptr)
        {
            qWarning() << "FATAL: Corrupt entitydata. Wrong type (not a path)";
            return;
        }
        QmlPathGeometry *pointcloudGeometry = static_cast<QmlPathGeometry *>(geometry());
        // TODO: does shared pointer survive here? Will the pointer be cleaned?
        QMetaObject::invokeMethod(pointcloudGeometry, "setPath", Qt::QueuedConnection, Q_ARG(PosePathPtr, path) );
    }
//#ifdef WITH_OPENVDB
    else if(strcmp(ed->type(), FloatGridEntitydata::TYPENAME()) == 0)
    {
        QGeometryRenderer::setPrimitiveType(QGeometryRenderer::Points);
        QGeometryRenderer::setGeometry(new QOpenVDBGridPointSurfaceGeometry(this));
        std::shared_ptr<openvdb::FloatGrid> grid = std::dynamic_pointer_cast< FloatGridEntitydata >(ed)->getData();
        if(grid == nullptr)
        {
            qWarning() << "FATAL: Corrupt entitydata. Wrong type (not a path)";
            return;
        }
        QOpenVDBGridPointSurfaceGeometry *gridGeometry = static_cast<QOpenVDBGridPointSurfaceGeometry *>(geometry());
        //gridGeometry->setGenerationMethod(QOpenVDBGridPointSurfaceGeometry::PointGenerationMethod::MarchingCubesCenter);
        QOpenVDBGrid *qgrid = new QOpenVDBGrid(gridGeometry);
        // Ensure that grid (shared pointer) lives as long as gridGeometry and qgrid!
        qgrid->setGrid(grid);
        gridGeometry->setGrid(qgrid);
    }
//#endif
    else
    {
        qDebug() << "unknown entitytype for visualization";
    }
}

QmlPointcloudCoordinatesystem *QmlEntitydataRenderer::coordinateSystem() const
{
    return m_coordinateSystem;
}

//Q_DECLARE_METATYPE(mapit::msgs::PosePathPtr)
