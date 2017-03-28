#ifndef RENDERMODEL_H
#define RENDERMODEL_H

#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QVector>

#include <upns/abstractentitydata.h>
#include <upns/ui/bindings/qmlentitydata.h>
#include <upns/ui/bindings/renderdata.h>

#include <pcl/PCLPointCloud2.h> //< Note: Only for workaround boost::serialization
#include <pcl/point_types.h> //< Note: Only for workaround boost::serialization

class Rendermodel
{
public:
    const std::string & getName() const { return m_sModelName; }

private:
    GLuint m_glPrimitveType;
    QOpenGLBuffer m_glVertexBuffer;
    QOpenGLBuffer m_glIndexBuffer;
    QOpenGLVertexArrayObject m_glVertexArray;
    //GLuint m_glTexture;
    GLsizei m_vertexCount;
    std::string m_modelName;
};

#endif
