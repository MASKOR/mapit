#ifndef QPOINTCLOUDGEOMETRY_H
#define QPOINTCLOUDGEOMETRY_H

#include <Qt3DRender/qgeometry.h>
#include "pcl/PCLPointCloud2.h"

class QPointcloudGeometryPrivate;

class QPointcloudGeometry : public Qt3DRender::QGeometry
{
    Q_OBJECT
    Q_PROPERTY(pcl::PCLPointCloud2 *pointcloud READ pointcloud WRITE setPointcloud NOTIFY pointcloudChanged)

public:
    explicit QPointcloudGeometry(QNode *parent = nullptr);
    ~QPointcloudGeometry();
    void updateVertices();

    pcl::PCLPointCloud2 *pointcloud() const;

public Q_SLOTS:
    void setPointcloud(pcl::PCLPointCloud2 * pointcloud);

Q_SIGNALS:
    void pointcloudChanged(pcl::PCLPointCloud2 * pointcloud);

private:
    QPointcloudGeometryPrivate *m_p;
};


#endif
