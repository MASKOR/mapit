#include "mapsrenderer.h"

#include <QtMath>
#include <QOpenGLFunctions_4_1_Core>
#include "upns_globals.h"
#include "libs/upns_interface/services.pb.h"
#include "libs/mapmanager/include/versioning/checkout.h"
#include "libs/layertypes_collection/pointcloud2/include/pointcloudlayer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

MapsRenderer::MapsRenderer()
    :m_entitydata( NULL ),
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
    program1.enableAttributeArray(colorAttr1);
    program1.setAttributeArray(vertexAttr1, vertices.constData());
    program1.setAttributeArray(normalAttr1, normals.constData());
    program1.setAttributeArray(colorAttr1, colors.constData());
    glDrawArrays(GL_POINTS, 0, vertices.size());
    program1.disableAttributeArray(colorAttr1);
    program1.disableAttributeArray(normalAttr1);
    program1.disableAttributeArray(vertexAttr1);
}


void MapsRenderer::initialize()
{
    initializeOpenGLFunctions();

    glClearColor(1.0f, 0.1f, 0.2f, 1.0f);

    QOpenGLFunctions_4_1_Core funcs;
    funcs.initializeOpenGLFunctions();
    //funcs.glPointSize(8.f);
    funcs.glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    funcs.glEnable(GL_POINT_SPRITE); // for gl_PointCoord. If not enabled, gl_PointCoord will always be 0

    QOpenGLShader *vshader1 = new QOpenGLShader(QOpenGLShader::Vertex, &program1);
    vshader1->compileSourceFile("resources/shader/pointcloud.vs");

    QOpenGLShader *fshader1 = new QOpenGLShader(QOpenGLShader::Fragment, &program1);
    fshader1->compileSourceFile("resources/shader/pointcloud.fs");

    program1.addShader(vshader1);
    program1.addShader(fshader1);
    program1.link();

    vertexAttr1 = program1.attributeLocation("vertex");
    normalAttr1 = program1.attributeLocation("normal");
    colorAttr1 = program1.attributeLocation("color");

    matrixUniformModelView = program1.uniformLocation("modelviewmatrix");
    matrixUniformProj = program1.uniformLocation("projectionmatrix");
    matrixUniformProjInv = program1.uniformLocation("projectioninvmatrix");
    matrixUniformModelViewNormal = program1.uniformLocation("modelviewnormalmatrix");
    screenSizeUniform = program1.uniformLocation("viewport");

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

    if( m_entitydata != NULL )
    {
        createGeometry();
    }
    m_initialized = true;
}

bool MapsRenderer::isInitialized()
{
    return m_initialized;
}

void MapsRenderer::setEntityData(upns::upnsSharedPointer<upns::AbstractEntityData> entityData)
{
    m_entitydata = entityData;
    if(m_initialized && m_entitydata != NULL)
    {
        createGeometry();
    }
}

void MapsRenderer::setMatrix(const QMatrix4x4 &mat)
{
    m_matrix = mat;
}

void MapsRenderer::reload()
{
    if(m_initialized)
    {
        createGeometry();
    }
}

void MapsRenderer::setScreenSize(const QSizeF &size)
{
    m_screenSize = size;
}

void MapsRenderer::render(const QMatrix4x4 &view, const QMatrix4x4 &proj)
{
    if(!m_initialized) return;
    glDepthMask(true);

    glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

    glFrontFace(GL_CW);
    glCullFace(GL_FRONT);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    program1.bind();
    QMatrix4x4 modelview(view*m_matrix);
    program1.setUniformValue(matrixUniformModelView, modelview);
    program1.setUniformValue(matrixUniformProj, proj);
    program1.setUniformValue(matrixUniformProjInv, proj.inverted());
    program1.setUniformValue(matrixUniformModelViewNormal, modelview.normalMatrix());
    program1.setUniformValue(screenSizeUniform, m_screenSize);
    drawPointcloud();
    program1.release();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
}

void MapsRenderer::createGeometry()
{
    assert(m_entitydata != NULL);
    vertices.clear();
    normals.clear();
    colors.clear();

    upns::upnsSharedPointer<PointcloudEntitydata> pcdData = upns::static_pointer_cast<PointcloudEntitydata>(m_entitydata);
    upnsPointcloud2Ptr pc2 = pcdData->getData();


    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::fromPCLPointCloud2(*pc2, *cloud);

    vertices.reserve(cloud->points.size());
    normals.reserve(cloud->points.size());
    colors.reserve(cloud->points.size());
    for(int i=0; i<cloud->points.size(); i++)
    {
      const pcl::PointXYZRGBNormal& pt = cloud->points[i];
      vertices << QVector3D(pt.data[0], pt.data[1], pt.data[2]);
      normals << QVector3D(pt.normal_x, pt.normal_y, pt.normal_z);
      colors << QVector3D(pt.r/255.f, pt.g/255.f, pt.b/255.f);
    }
    qDebug() << "Entity loaded:" << vertices.size() << "points.";
}
