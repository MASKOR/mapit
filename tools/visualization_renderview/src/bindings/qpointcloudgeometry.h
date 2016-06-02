#ifndef QPOINTCLOUDGEOMETRY_H
#define QPOINTCLOUDGEOMETRY_H

#include <Qt3DRender/qgeometry.h>
#include "pcl/PCLPointCloud2.h"


namespace Qt3DRender {

class QAttribute;

} // Qt3DRender

class QPointcloudGeometryPrivate;

class QPointcloudGeometry : public Qt3DRender::QGeometry
{
    Q_OBJECT
    Q_PROPERTY(pcl::PCLPointCloud2 *pointcloud READ pointcloud WRITE setPointcloud NOTIFY pointcloudChanged)
    Q_PROPERTY(Qt3DRender::QAttribute *positionAttribute READ positionAttribute CONSTANT)
    Q_PROPERTY(Qt3DRender::QAttribute *normalAttribute READ normalAttribute CONSTANT)
    Q_PROPERTY(Qt3DRender::QAttribute *texCoordAttribute READ texCoordAttribute CONSTANT)
    Q_PROPERTY(Qt3DRender::QAttribute *indexAttribute READ indexAttribute CONSTANT)

public:
    explicit QPointcloudGeometry(QNode *parent = nullptr);
    ~QPointcloudGeometry();
    void updateVertices();

    pcl::PCLPointCloud2 *pointcloud() const;

    Qt3DRender::QAttribute *positionAttribute() const;
    Qt3DRender::QAttribute *normalAttribute() const;
    Qt3DRender::QAttribute *texCoordAttribute() const;
    Qt3DRender::QAttribute *indexAttribute() const;

public Q_SLOTS:
    void setPointcloud(pcl::PCLPointCloud2 * pointcloud);

Q_SIGNALS:
    void pointcloudChanged(pcl::PCLPointCloud2 * pointcloud);

private:
    QPointcloudGeometryPrivate *m_p;
};


#endif
