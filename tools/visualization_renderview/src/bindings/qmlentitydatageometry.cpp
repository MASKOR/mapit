#include "qmlentitydatageometry.h"
#include "abstractentitydata.h"
#include "libs/layertypes_collection/pointcloud2/include/pointcloudlayer.h"

Qt3DRender::QGeometry *QmlEntitydataGeometry::geometry() const
{
    return m_geometry;
}

QmlEntitydata *QmlEntitydataGeometry::entitydata() const
{
    return m_entitydata;
}

void QmlEntitydataGeometry::setEntitydata(QmlEntitydata *entitydata)
{
    if (m_entitydata == entitydata)
        return;

    m_entitydata = entitydata;
    updateGeometry();
    Q_EMIT entitydataChanged(entitydata);
}

void QmlEntitydataGeometry::updateGeometry()
{
    upns::upnsSharedPointer<upns::AbstractEntityData> ed = m_entitydata->getEntityData();

    if(m_geometry == NULL)
    {
        m_geometry = new Qt3DRender::QGeometry();
    }
    else
    {
        m_geometry->
    }
    switch(ed->layerType())
    {
    case upns::POINTCLOUD2:
        createGeometryFromPointcloud( upns::static_pointer_cast< PointcloudEntitydata >(ed), m_geometry );
        Q_EMIT geometryChanged(m_geometry);
        break;
    case upns::OCTOMAP:
        break;
    case upns::OPENVDB:
        break;
    case upns::POSES:
        break;
    case upns::NONE:
    default:
        break;
    }
}


Qt3DRender::QGeometry *createGeometryFromPointcloud( upns::upnsSharedPointer< upns::PointcloudEntitydata > pcd, Qt3DRender::QGeometry *out )
{
    upnsPointcloud2Ptr pc2 = pcdData->getData();
    if(out)
}
