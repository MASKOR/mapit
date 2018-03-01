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

#ifndef VRTHREAD_H
#define VRTHREAD_H

#include <QOpenGLFramebufferObject>
#include <QOffscreenSurface>
#include <QMatrix4x4>
#include <QThread>
#include "upns/ui/bindings/qmlentitydata.h"
#include <QOpenGLDebugMessage>

#ifdef VRMODE
#include <OVR_CAPI_GL.h>
#endif
class MapsRenderer;

class TextureBuffer;
class DepthBuffer;
/*
 * TODO: Get new independent OpenGL Context and use shared texture
 */
class VRThread : public QThread
{
    Q_OBJECT

public:
    VRThread(const QSize &size);
    virtual ~VRThread();
};

#endif // MapsRenderer_H
