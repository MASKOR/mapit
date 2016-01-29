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
    void setPointSize(const float size);
    void setFilename(const QString &filename);
private:
    QMatrix4x4   m_matrix;

    void drawPointcloud();
    void createGeometry();
    void createGeometry(QString filename);

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
    int pointSizeUniform;

    bool m_initialized;
    upns::upnsSharedPointer<upns::AbstractEntityData> m_entitydata;
    //Note: Filenames are a workaround, as long as stubs are used and as long as boost::serialization is used and too slow
    QString m_filename;
    QSizeF m_screenSize;

};

#endif
