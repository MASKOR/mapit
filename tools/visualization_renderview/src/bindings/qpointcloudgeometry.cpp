#include "qpointcloudgeometry.h"
#include "Qt3DRender/QBuffer"

class QPointcloudGeometryPrivate
{
    Qt3DRender::QBuffer *m_vertexBuffer;
    pcl::PCLPointCloud2 *m_pointcloud;
};

QPointcloudGeometry::QPointcloudGeometry(Qt3DCore::QNode *parent)
    :m_p(new QPointcloudGeometryPrivate)
{

}

QPointcloudGeometry::~QPointcloudGeometry()
{
    delete m_p;
}

void QPointcloudGeometry::updateVertices()
{
    for(int field=0 ; m_p->m_pointcloud->fields.size() ; ++i)
    {

    }
}

pcl::PCLPointCloud2 *QPointcloudGeometry::pointcloud() const
{
    return m_p->m_pointcloud;
}

void Qt3DExtras::QPointcloudGeometry::setPointcloud(pcl::PCLPointCloud2 *pointcloud)
{
    if (m_pointcloud == pointcloud)
        return;

    m_pointcloud = pointcloud;
    updateVertices();
    emit pointcloudChanged(pointcloud);
}
