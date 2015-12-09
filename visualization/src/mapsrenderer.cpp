#include "mapsrenderer.h"

#include <QtMath>
#include "upns_globals.h"
#include "upns_interface/services.pb.h"
#include "mapmanager/src/mapmanager.h"
#include "layertypes/pointcloud2/include/pointcloudlayer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

MapsRenderer::MapsRenderer()
    :m_mapManager(NULL),
      m_mapId( 0 ),
      m_initialized( false )
{
}

MapsRenderer::~MapsRenderer()
{
}


void MapsRenderer::drawPointcloud()
{
    program1.enableAttributeArray(normalAttr1);
    program1.enableAttributeArray(vertexAttr1);
    program1.setAttributeArray(vertexAttr1, vertices.constData());
    //program1.setAttributeArray(normalAttr1, normals.constData());
    glDrawArrays(GL_POINTS, 0, vertices.size());
    program1.disableAttributeArray(normalAttr1);
    program1.disableAttributeArray(vertexAttr1);
}


void MapsRenderer::initialize()
{
    initializeOpenGLFunctions();

    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);

    QOpenGLShader *vshader1 = new QOpenGLShader(QOpenGLShader::Vertex, &program1);
    const char *vsrc1 =
        "attribute highp vec4 vertex;\n"
        "attribute mediump vec3 normal;\n"
        "uniform mediump mat4 matrix;\n"
        "varying mediump vec4 color;\n"
        "void main(void)\n"
        "{\n"
        "    vec3 toLight = normalize(vec3(0.0, 0.3, 1.0));\n"
        "    float angle = max(dot(toLight, toLight), 0.0);\n"
        "    vec3 col = vec3(0.40, 1.0, 0.0);\n"
        "    color = vec4(col * 0.2 + col * 0.8 * angle, 1.0);\n"
        "    color = clamp(color, 0.0, 1.0);\n"
        "    gl_Position = matrix * vertex;\n"
        "}\n";
    vshader1->compileSourceCode(vsrc1);

    QOpenGLShader *fshader1 = new QOpenGLShader(QOpenGLShader::Fragment, &program1);
    const char *fsrc1 =
        "varying mediump vec4 color;\n"
        "void main(void)\n"
        "{\n"
        "    gl_FragColor = color;\n"
        "}\n";
    fshader1->compileSourceCode(fsrc1);

    program1.addShader(vshader1);
    program1.addShader(fshader1);
    program1.link();

    vertexAttr1 = program1.attributeLocation("vertex");
    normalAttr1 = program1.attributeLocation("normal");
    matrixUniform1 = program1.uniformLocation("matrix");

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

    m_fAngle = 0;
    m_fScale = 1;
    if(m_mapManager && m_mapId != 0)
    {
        createGeometry();
    }
    m_initialized = true;
}

void MapsRenderer::setMapmanager(upns::MapManager *mapman)
{
    m_mapManager = mapman;
    if(m_initialized)
    {
        createGeometry();
    }
}

void MapsRenderer::setMapId(upns::MapIdentifier mapId)
{
    m_mapId = mapId;
    if(m_initialized)
    {
        createGeometry();
    }
}

void MapsRenderer::render()
{
    if(!m_initialized) return;
    glDepthMask(true);

    glClearColor(0.5f, 0.5f, 0.7f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

    glFrontFace(GL_CW);
    glCullFace(GL_FRONT);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    QMatrix4x4 modelview;
    modelview.rotate(m_fAngle, 0.0f, 1.0f, 0.0f);
    modelview.rotate(m_fAngle, 1.0f, 0.0f, 0.0f);
    modelview.rotate(m_fAngle, 0.0f, 0.0f, 1.0f);
    modelview.scale(m_fScale);
    modelview.translate(0.0f, -0.2f, 0.0f);

    program1.bind();
    program1.setUniformValue(matrixUniform1, modelview);
    drawPointcloud();
    program1.release();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    m_fAngle += 1.0f;
}

void MapsRenderer::createGeometry()
{
    assert(m_mapManager);
    vertices.clear();
    normals.clear();

    upns::upnsSharedPointer<upns::Map> map = m_mapManager->getInternalMapService()->getMap(m_mapId);
    upns::upnsSharedPointer<upns::AbstractEntityData> aed = m_mapManager->getInternalMapService()->getEntityData(m_mapId, map->layers(0).id(), map->layers(0).entities(0).id());
    upns::upnsSharedPointer<PointcloudEntitydata> pcdData = upns::static_pointer_cast<PointcloudEntitydata>(aed);
    upnsPointcloud2Ptr pc2 = pcdData->getData();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pc2, *cloud);

    vertices.reserve(cloud->points.size());
    for(int i=0; i<cloud->points.size(); i++)
    {
      const pcl::PointXYZ& pt = cloud->points[i];
      vertices << QVector3D(pt.data[0], pt.data[1], pt.data[2]);
    }
    qDebug() << "Map loaded:" << vertices.size() << "points.";
}
