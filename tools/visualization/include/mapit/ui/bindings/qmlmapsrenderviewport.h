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

#ifndef MapsRenderer_H
#define MapsRenderer_H

#include <QQuickItem>
#include <QMatrix4x4>
#include "qmlentitydata.h"
#include "renderdata.h"
#include <QTransform>

class RenderThread;

class QmlMapsRenderViewport : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(Renderdata *renderdata READ renderdata NOTIFY renderdataChanged)
    Q_PROPERTY(QMatrix4x4 finalTransform READ finalTransform NOTIFY finalTransformChanged)
public:
    QmlMapsRenderViewport();

    static QList<QThread *> threads;

    Renderdata *renderdata()
    {
        return &m_renderdata;
    }

    QMatrix4x4 finalTransform() const;

public Q_SLOTS:
    void ready();
    //TODO void reload();
    void emitFrame();

Q_SIGNALS:
    void frame();

    void renderdataChanged(Renderdata * renderdata);

    void finalTransformChanged();

protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);

private Q_SLOTS:

private:
    RenderThread *m_renderThread;

    std::shared_ptr<QMetaObject::Connection> m_connectionToEntitydata;
    Renderdata m_renderdata;
};

#endif // MapsRenderer_H
