#ifndef QMLENTITYDATAGEOMETRY_H
#define QMLENTITYDATAGEOMETRY_H

#include <Qt3DRender/QGeometryRenderer>
#include "qmlentitydata.h"

class QmlEntitydataRenderer : public Qt3DRender::QGeometryRenderer
{
    Q_OBJECT
    Q_PROPERTY(QmlEntitydata* entitydata READ entitydata WRITE setEntitydata NOTIFY entitydataChanged)

public:
    explicit QmlEntitydataRenderer(Qt3DCore::QNode *parent = Q_NULLPTR);
    QmlEntitydata* entitydata() const;

    Q_INVOKABLE void updateGeometry();

public Q_SLOTS:
    void setEntitydata(QmlEntitydata* entitydata);

Q_SIGNALS:
    void entitydataChanged(QmlEntitydata* entitydata);

private:
    QmlEntitydata* m_entitydata;

    void setInstanceCount(int instanceCount);
    void setPrimitiveCount(int primitiveCount);
    void setBaseVertex(int baseVertex);
    void setBaseInstance(int baseInstance);
    void setRestartIndex(int index);
    void setPrimitiveRestart(bool enabled);
    void setGeometry(Qt3DRender::QGeometry *geometry);
    void setPrimitiveType(PrimitiveType primitiveType);
};

#endif
