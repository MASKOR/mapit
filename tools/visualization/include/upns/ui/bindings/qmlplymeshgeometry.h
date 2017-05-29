#ifndef QMLPLAYMESHGEOMETRY_H
#define QMLPLAYMESHGEOMETRY_H

#include <Qt3DRender/qgeometry.h>
#include <upns/layertypes/assettype.h>

class QmlPlyMeshGeometryPrivate;

class QmlPlyMeshGeometry : public Qt3DRender::QGeometry
{
    Q_OBJECT
public:
    explicit QmlPlyMeshGeometry(QNode *parent = NULL);
    ~QmlPlyMeshGeometry();
    void updateVertices();

public Q_SLOTS:
    void setAsset(AssetPtr asset);
private Q_SLOTS:
    void updateAttributes(uint32_t vertexCount);
Q_SIGNALS:
    void assetChanged(AssetPtr asset);
private:
    QmlPlyMeshGeometryPrivate *m_p;
};


#endif
