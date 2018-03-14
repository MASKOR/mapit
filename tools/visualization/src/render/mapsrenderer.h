/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2015 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef MAPSRENDERER_H
#define MAPSRENDERER_H

#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QVector>

#include <mapit/abstractentitydata.h>
#include "mapit/ui/bindings/qmlentitydata.h"
#include "mapit/ui/bindings/renderdata.h"

#include <pcl/PCLPointCloud2.h> //< Note: Only for workaround boost::serialization

class MapsRenderer : protected QOpenGLFunctions
{
public:
    MapsRenderer(Renderdata *renderdata);
    ~MapsRenderer();

    void render(const QMatrix4x4 &view, const QMatrix4x4 &proj, QVector4D &viewportSize, float heightOfNearPlane);
    void initialize();
    bool isInitialized();

//    void setEntitydata(std::shared_ptr<mapit::AbstractEntitydata> entityData);
//    void setMatrix( const QMatrix4x4 &mat );
    void reload();
//    void setScreenSize(const QSizeF &size);
//    void setPointSize(const float size);
//    void setDistanceDetail(const float detail);
//    void setFilename(const QString &filename);
private:
//    QMatrix4x4   m_matrix;

    void drawPointcloud();
    void createGeometry();
    void createGeometry(QString filename);
    void createGeometry(pcl::PCLPointCloud2 &pointcoud);

    QVector<QVector3D> vertices;
    QVector<QVector3D> normals;
    QVector<QVector3D> colors;
    QOpenGLShaderProgram m_shaderProgram;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_positionBuffer;
    QOpenGLBuffer m_colorBuffer;
    QOpenGLBuffer m_normalBuffer;

    int m_positionAttr;
    int m_normalAttr;
    int m_colorAttr;
    int matrixUniformModelView;
    int matrixUniformProj;
    int matrixUniformProjInv;
    int matrixUniformModelViewNormal;
    int screenSizeUniform;
    int pointSizeUniform;
    int heightOfNearPlaneUniform;
    int discUniform;
    int distanceDetailUniform;

    bool m_initialized;
    std::shared_ptr<mapit::AbstractEntitydata> m_entitydata;
//    //Note: Filenames are a workaround, as long as stubs are used and as long as boost::serialization is used and too slow
//    QString m_filename;
//    QSizeF m_screenSize;
    unsigned int m_pointcloudSize;
    Renderdata *m_renderdata;
};

#endif
