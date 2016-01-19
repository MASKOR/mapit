#ifndef MapsRenderer_H
#define MapsRenderer_H

#include <QQuickItem>
#include <QMatrix4x4>
#include "bindings/qmlentitydata.h"

class RenderThread;

class QmlMapsRenderViewport : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(QmlEntitydata *entitydata READ entitydata WRITE setEntitydata NOTIFY entitydataChanged)
    Q_PROPERTY(QMatrix4x4 matrix READ matrix WRITE setMatrix NOTIFY matrixChanged)

public:
    QmlMapsRenderViewport();

    static QList<QThread *> threads;

    QMatrix4x4 matrix() const
    {
        return m_matrix;
    }

    QmlEntitydata * entitydata() const
    {
        return m_entitydata;
    }

public Q_SLOTS:
    void ready();
    void reload();

    void setMatrix(QMatrix4x4 matrix)
    {
        if (m_matrix == matrix)
            return;

        m_matrix = matrix;
        Q_EMIT matrixChanged(matrix);
    }

    void setEntitydata(QmlEntitydata * entitydata)
    {
        if (m_entitydata == entitydata)
            return;
        if(m_entitydata)
        {
//            if(m_connectionToEntityData != NULL)
//            {
//                disconnect(*m_connectionToEntityData);
//                m_connectionToEntityData = NULL;
//            }
            disconnect(m_entitydata, &QmlEntitydata::updated, this, &QmlMapsRenderViewport::entitydataChanged);
        }
        m_entitydata = entitydata;
        if(m_entitydata)
        {
//            upns::upnsSharedPointer<QMetaObject::Connection> con( new QMetaObject::Connection(
//                        connect(m_entitydata,
//                                &QmlEntitydata::updated,
//                                this,
//                                [&](){this->entitydataChanged(m_entitydata);})));
//            m_connectionToEntityData = con;
            connect(m_entitydata, &QmlEntitydata::updated, this, &QmlMapsRenderViewport::entitydataChanged);
        }
        Q_EMIT entitydataChanged(entitydata);
    }

Q_SIGNALS:
    void needsReload();

    void matrixChanged(QMatrix4x4 matrix);

    void entitydataChanged(QmlEntitydata * entitydata);

protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);

private:
    RenderThread *m_renderThread;
    QMatrix4x4 m_matrix;
    QmlEntitydata * m_entitydata;
    //upns::upnsSharedPointer<QMetaObject::Connection> m_connectionToEntityData;
};

#endif // MapsRenderer_H
