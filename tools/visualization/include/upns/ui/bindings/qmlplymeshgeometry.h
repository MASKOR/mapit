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
    void setAsset(upnsAssetPtr asset);
private Q_SLOTS:
    void updateAttributes();
Q_SIGNALS:
    void assetChanged(upnsAssetPtr asset);
private:
    QmlPlyMeshGeometryPrivate *m_p;
};


#endif
