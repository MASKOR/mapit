#ifndef MAPSRENDERER_H
#define MAPSRENDERER_H

#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QVector>

#include "mapmanager/src/mapmanager.h"

class MapsRenderer : protected QOpenGLFunctions
{
public:
    MapsRenderer();
    ~MapsRenderer();

    void render();
    void initialize();
    bool isInitialized();

    void setMapmanager(upns::MapManager *mapman);
    void setMapId( upns::MapIdentifier mapId);
    void setLayerId(upns::LayerIdentifier layerId);
    void setMatrix( const QMatrix4x4 &mat );
    void reloadMap();
private:
    QMatrix4x4   m_matrix;

    void drawPointcloud();
    void createGeometry();

    QVector<QVector3D> vertices;
    QVector<QVector3D> normals;
    QOpenGLShaderProgram program1;
    int vertexAttr1;
    int normalAttr1;
    int matrixUniform1;

    upns::MapManager *m_mapManager;
    upns::MapIdentifier m_mapId;
    upns::LayerIdentifier m_layerId;
    bool m_initialized;
};

#endif
