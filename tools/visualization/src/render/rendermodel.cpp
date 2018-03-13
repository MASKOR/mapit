/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

//#include "mapsrenderer.h"

//#include <QtMath>
//#include <QOpenGLFunctions_3_0>

//#include "upns/ui/bindings/qmlentitydata.h"
//#include <upns/logging.h>
//#include <mapit/msgs/services.pb.h>
//#include <upns/versioning/checkout.h>
//#include <upns/layertypes/pointcloudlayer.h>

//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>

//MapsRenderer::MapsRenderer(Renderdata *renderdata)
//    :m_initialized( false ),
//     m_entitydata( NULL ),
//     m_pointcloudSize(0),
//     m_renderdata(renderdata)
//{
//    QObject::connect(m_renderdata, &Renderdata::entitydataChanged, [&](QmlEntitydata *entitydata){
//        m_entitydata = entitydata->getEntitydata();
//        if(m_initialized && m_entitydata != NULL)
//        {
//            createGeometry();
//        }
//    });
//    QObject::connect(m_renderdata, &Renderdata::filenameChanged, [&](QString filename){
//        if(m_initialized)
//        {
//            createGeometry(filename);
//        }
//    });
//}

//MapsRenderer::~MapsRenderer()
//{
//}


//void MapsRenderer::drawPointcloud()
//{
//    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
//    glDrawArrays(GL_POINTS, 0, m_pointcloudSize);
//    //glDrawElements(GL_POINTS, m_pointcloudSize, GL_UNSIGNED_SHORT, NULL);
////    program1.enableAttributeArray(normalAttr1);
////    program1.enableAttributeArray(vertexAttr1);
////    program1.enableAttributeArray(colorAttr1);
////    program1.setAttributeArray(vertexAttr1, vertices.constData());
////    program1.setAttributeArray(normalAttr1, normals.constData());
////    program1.setAttributeArray(colorAttr1, colors.constData());
////    glDrawArrays(GL_POINTS, 0, vertices.size());
////    program1.disableAttributeArray(colorAttr1);
////    program1.disableAttributeArray(normalAttr1);
////    program1.disableAttributeArray(vertexAttr1);
//}


//void MapsRenderer::initialize()
//{
//    initializeOpenGLFunctions();

//    glClearColor(1.0f, 0.1f, 0.2f, 1.0f);

//    QOpenGLFunctions_3_0 funcs;
//    funcs.initializeOpenGLFunctions();
//    //funcs.glPointSize(8.f);
//    funcs.glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
//    funcs.glEnable(GL_POINT_SPRITE); // for gl_PointCoord. If not enabled, gl_PointCoord will always be 0

//    QOpenGLShader *vshader1 = new QOpenGLShader(QOpenGLShader::Vertex, &m_shaderProgram);
//    vshader1->compileSourceFile("resources/shader/pointcloud.vs");

//    QOpenGLShader *fshader1 = new QOpenGLShader(QOpenGLShader::Fragment, &m_shaderProgram);
//    fshader1->compileSourceFile("resources/shader/pointcloud.fs");

//    m_shaderProgram.addShader(vshader1);
//    m_shaderProgram.addShader(fshader1);
//    m_shaderProgram.link();

//    m_positionAttr = m_shaderProgram.attributeLocation("vertex");
//    m_normalAttr = m_shaderProgram.attributeLocation("normal");
//    m_colorAttr = m_shaderProgram.attributeLocation("color");

//    matrixUniformModelView = m_shaderProgram.uniformLocation("modelviewmatrix");
//    matrixUniformProj = m_shaderProgram.uniformLocation("projectionmatrix");
//    matrixUniformProjInv = m_shaderProgram.uniformLocation("projectioninvmatrix");
//    matrixUniformModelViewNormal = m_shaderProgram.uniformLocation("modelviewnormalmatrix");
//    pointSizeUniform = m_shaderProgram.uniformLocation("pointsize");
//    discUniform = m_shaderProgram.uniformLocation("discrender");
//    screenSizeUniform = m_shaderProgram.uniformLocation("viewport");
//    distanceDetailUniform = m_shaderProgram.uniformLocation("distanceDetail");
//    heightOfNearPlaneUniform = m_shaderProgram.uniformLocation("heightOfNearPlaneInv");

//    m_shaderProgram.bind();
//    m_shaderProgram.setUniformValue(pointSizeUniform, 64.0f);
//    m_shaderProgram.setUniformValue(discUniform, static_cast<int>(0));
//    m_shaderProgram.setUniformValue(distanceDetailUniform, 0.05f);
//    m_shaderProgram.release();

//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

//    if( m_renderdata->entitydata() != NULL && m_renderdata->entitydata()->getEntitydata() != NULL )
//    {
//        m_entitydata = m_renderdata->entitydata()->getEntitydata();
//        createGeometry();
//    }
//    else // else workaround fast pcd loading
//    {
//        createGeometry(m_renderdata->filename());
//    }
//    m_initialized = true;
//}

//bool MapsRenderer::isInitialized()
//{
//    return m_initialized;
//}

////void MapsRenderer::setEntitydata(std::shared_ptr<upns::AbstractEntitydata> entityData)
////{
////    m_entitydata = entityData;
////    if(m_initialized && m_entitydata != NULL)
////    {
////        createGeometry();
////    }
////}

////void MapsRenderer::setMatrix(const QMatrix4x4 &mat)
////{
////    m_matrix = mat;
////}

//void MapsRenderer::reload()
//{
//    if(m_initialized)
//    {
//        createGeometry();
//    }
//}

////void MapsRenderer::setScreenSize(const QSizeF &size)
////{
////    m_screenSize = size;
////}

////void MapsRenderer::setPointSize(const float size)
////{
////    m_shaderProgram.bind();
////    m_shaderProgram.setUniformValue(pointSizeUniform, m_renderdata->pointSize());
////    m_shaderProgram.release();
////}

////void MapsRenderer::setDistanceDetail(const float detail)
////{
////    m_shaderProgram.bind();
////    m_shaderProgram.setUniformValue(distanceDetailUniform, m_renderdata->distanceDetail());
////    m_shaderProgram.release();
////}

////void MapsRenderer::setFilename(const QString &filename)
////{
////    m_filename = filename;
////    if(m_initialized)
////    {
////        createGeometry(filename);
////    }
////}

//void MapsRenderer::render(const QMatrix4x4 &view, const QMatrix4x4 &proj, QVector4D &viewportSize, float heightOfNearPlane)
//{
//    if(!m_initialized) return;
//    glDepthMask(true);

//    glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
//    //glClearColor(1.f, 1.f, 1.0f, 1.0f);
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

//    glFrontFace(GL_CW);
//    glCullFace(GL_FRONT);
//    glEnable(GL_CULL_FACE);
//    glEnable(GL_DEPTH_TEST);
////    glEnable(GL_BLEND);
////    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//    m_shaderProgram.bind();
//    QMatrix4x4 modelview(view*m_renderdata->matrix());
//    m_shaderProgram.setUniformValue(pointSizeUniform, static_cast<float>(m_renderdata->pointSize()));
//    m_shaderProgram.setUniformValue(discUniform, static_cast<int>(m_renderdata->disc()));
//    m_shaderProgram.setUniformValue(distanceDetailUniform, static_cast<float>(m_renderdata->distanceDetail()));
//    m_shaderProgram.setUniformValue(matrixUniformModelView, modelview);
//    m_shaderProgram.setUniformValue(matrixUniformProj, proj);
//    m_shaderProgram.setUniformValue(matrixUniformProjInv, proj.inverted());
//    m_shaderProgram.setUniformValue(matrixUniformModelViewNormal, modelview.normalMatrix());
//    m_shaderProgram.setUniformValue(screenSizeUniform, viewportSize);
//    m_shaderProgram.setUniformValue(heightOfNearPlaneUniform, heightOfNearPlane);
//    drawPointcloud();
//    m_shaderProgram.release();

//    glDisable(GL_DEPTH_TEST);
//    glDisable(GL_CULL_FACE);
//}

//void MapsRenderer::createGeometry()
//{
//    if(m_entitydata == NULL)
//    {
//        // Workaround
//        createGeometry(m_renderdata->filename());
//        return;
//    }
//    assert(m_entitydata != NULL);

//    std::shared_ptr<PointcloudEntitydata> pcdData = std::d_pointer_cast<PointcloudEntitydata>(m_entitydata);
//    upnsPointcloud2Ptr pc2 = pcdData->getData();

//    createGeometry(*pc2);
//}

//void MapsRenderer::createGeometry(QString filename)
//{
//    if(filename.isEmpty()) return;
//    pcl::PCLPointCloud2 pc2;
//    if(filename.endsWith(".pcd"))
//    {
//        pcl::PCDReader reader;
//        if ( reader.read(filename.toStdString(), pc2) < 0 )
//        {
//            log_error("Couldn't read file" + filename.toStdString());
//            return;
//        }
//    }
//    else
//    {
//        pcl::PLYReader reader;
//        if ( reader.read(filename.toStdString(), pc2) < 0 )
//        {
//            log_error("Couldn't read file" + filename.toStdString());
//            return;
//        }
//    }
//    createGeometry(pc2);
//}

////void MapsRenderer::createGeometry(pcl::PCLPointCloud2 &pointcoud)
////{
////    vertices.clear();
////    normals.clear();
////    colors.clear();

////    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
////    pcl::fromPCLPointCloud2(pc2, *cloud);

////    vertices.reserve(cloud->points.size());
////    normals.reserve(cloud->points.size());
////    colors.reserve(cloud->points.size());
////    for(int i=0; i<cloud->points.size(); i++)
////    {
////        const pcl::PointXYZRGBNormal& pt = cloud->points[i];
////        vertices << QVector3D(pt.data[0], pt.data[1], pt.data[2]);
////        normals << QVector3D(pt.normal_x, pt.normal_y, pt.normal_z);
////        colors << QVector3D(pt.r/255.f, pt.g/255.f, pt.b/255.f);
////    }
////    qDebug() << "Entity loaded:" << vertices.size() << "points.";
////}

//void MapsRenderer::createGeometry(pcl::PCLPointCloud2 &pointcoud)
//{
//    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//    pcl::fromPCLPointCloud2(pointcoud, *cloud);

//    m_pointcloudSize = cloud->points.size();
//    // Create VAO for first object to render
//    m_vao.create();
//    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);

//    // Setup VBOs and IBO (use QOpenGLBuffer to buffer data,
//    // specify format, usage hint etc). These will be
//    // remembered by the currently bound VAO
//    m_positionBuffer.create();
//    m_positionBuffer.setUsagePattern( QOpenGLBuffer::StaticDraw );
//    m_positionBuffer.bind();
//    m_positionBuffer.allocate( m_pointcloudSize * 3 * sizeof( float ) );
//    m_shaderProgram.enableAttributeArray( m_positionAttr );
//    m_shaderProgram.setAttributeBuffer( m_positionAttr, GL_FLOAT, 0, 3 );
//    GLfloat *positions = static_cast<GLfloat*>(m_positionBuffer.map( QOpenGLBuffer::WriteOnly ));

//    m_colorBuffer.create();
//    m_colorBuffer.setUsagePattern( QOpenGLBuffer::StaticDraw );
//    m_colorBuffer.bind();
//    m_colorBuffer.allocate( m_pointcloudSize * 3 * sizeof( float ) );
//    m_shaderProgram.enableAttributeArray( m_colorAttr );
//    m_shaderProgram.setAttributeBuffer( m_colorAttr, GL_FLOAT, 0, 3 );
//    GLfloat *colors = static_cast<GLfloat*>(m_colorBuffer.map( QOpenGLBuffer::WriteOnly ));

//    m_normalBuffer.create();
//    m_normalBuffer.setUsagePattern( QOpenGLBuffer::StaticDraw );
//    m_normalBuffer.bind();
//    m_normalBuffer.allocate( m_pointcloudSize * 3 * sizeof( float ) );
//    m_shaderProgram.enableAttributeArray( m_normalAttr );
//    m_shaderProgram.setAttributeBuffer( m_normalAttr, GL_FLOAT, 0, 3 );
//    GLfloat *normals = static_cast<GLfloat*>(m_normalBuffer.map( QOpenGLBuffer::WriteOnly ));

//    for(int i=0; i<m_pointcloudSize; i++)
//    {
//        const pcl::PointXYZRGBNormal& pt = cloud->points[i];
//        positions[i*3  ] = pt.data[0];
//        positions[i*3+1] = pt.data[1];
//        positions[i*3+2] = pt.data[2];
//        normals[i*3  ] = pt.normal_x;
//        normals[i*3+1] = pt.normal_y;
//        normals[i*3+2] = pt.normal_z;
//        colors[i*3  ] = pt.r/255.f;
//        colors[i*3+1] = pt.g/255.f;
//        colors[i*3+2] = pt.b/255.f;
//    }
//    m_positionBuffer.bind();
//    m_positionBuffer.unmap();
//    m_colorBuffer.bind();
//    m_colorBuffer.unmap();
//    m_normalBuffer.bind();
//    m_normalBuffer.unmap();

//    //m_normalBuffer.release();

//    qDebug() << "Entity loaded:" << m_pointcloudSize << "points.";
//}
