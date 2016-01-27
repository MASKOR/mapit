#ifndef MAPSRENDERER_H
#define MAPSRENDERER_H

#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QVector>

#include "abstractentitydata.h"
#include "bindings/qmlentitydata.h"

class MapsRenderer : protected QOpenGLFunctions
{
public:
    MapsRenderer();
    ~MapsRenderer();

    void render(const QMatrix4x4 &view, const QMatrix4x4 &proj);
    void initialize();
    bool isInitialized();

    void setEntityData(upns::upnsSharedPointer<upns::AbstractEntityData> entityData);
    void setMatrix( const QMatrix4x4 &mat );
    void reload();
    void setScreenSize(const QSizeF &size);
private:
    QMatrix4x4   m_matrix;

    void drawPointcloud();
    void createGeometry();

    QVector<QVector3D> vertices;
    QVector<QVector3D> normals;
    QVector<QVector3D> colors;
    QOpenGLShaderProgram program1;
    int vertexAttr1;
    int normalAttr1;
    int colorAttr1;
    int matrixUniformModelView;
    int matrixUniformProj;
    int matrixUniformProjInv;
    int matrixUniformModelViewNormal;
    int screenSizeUniform;

    bool m_initialized;
    upns::upnsSharedPointer<upns::AbstractEntityData> m_entitydata;
    QSizeF m_screenSize;
};

#endif
