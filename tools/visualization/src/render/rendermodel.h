/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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
