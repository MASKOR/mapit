#ifndef QMLPATHGEOMETRY_H
#define QMLPATHGEOMETRY_H

#include <Qt3DRender/qgeometry.h>
#include <upns/layertypes/pose_path.h>

class QmlPathGeometryPrivate;

class QmlPathGeometry : public Qt3DRender::QGeometry
{
    Q_OBJECT
public:
    explicit QmlPathGeometry(QNode *parent = NULL);
    ~QmlPathGeometry();
    void updateVertices();

public Q_SLOTS:
    void setPath(PosePathPtr path);
private Q_SLOTS:
    void updateAttributes();
Q_SIGNALS:
    void pathChanged(PosePathPtr path);
private:
    QmlPathGeometryPrivate *m_p;
};


#endif
