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

#ifndef RENDERDATA_H
#define RENDERDATA_H

#include <QMatrix4x4>
#include "qmlentitydata.h"

class QQuickItem;

/*
 * Renderdata must be shared across threads.
 */
class Renderdata : public QObject
{
    Q_OBJECT
    // Input from Script
    Q_PROPERTY(QmlEntitydata *entitydata READ entitydata WRITE setEntitydata NOTIFY entitydataChanged)
    Q_PROPERTY(qreal width READ width WRITE setWidth NOTIFY widthChanged)
    Q_PROPERTY(qreal height READ height WRITE setHeight NOTIFY heightChanged)
    Q_PROPERTY(QMatrix4x4 matrix READ matrix WRITE setMatrix NOTIFY matrixChanged)
    Q_PROPERTY(bool vrmode READ vrmode WRITE setVrmode NOTIFY vrmodeChanged)
    Q_PROPERTY(bool mirrorEnabled READ mirrorEnabled WRITE setMirrorEnabled NOTIFY mirrorEnabledChanged)
    Q_PROPERTY(bool mirrorDistorsion READ mirrorDistorsion WRITE setMirrorDistorsion NOTIFY mirrorDistorsionChanged)
    Q_PROPERTY(bool mirrorRightEye READ mirrorRightEye WRITE setMirrorRightEye NOTIFY mirrorRightEyeChanged)
    Q_PROPERTY(qreal pointSize READ pointSize WRITE setPointSize NOTIFY pointSizeChanged)
    Q_PROPERTY(qreal distanceDetail READ distanceDetail WRITE setDistanceDetail NOTIFY distanceDetailChanged)
    Q_PROPERTY(QString filename READ filename WRITE setFilename NOTIFY filenameChanged)
    // Output to Script
    Q_PROPERTY(QMatrix4x4 headMatrix READ headMatrix NOTIFY headMatrixChanged)
    Q_PROPERTY(QVector3D headDirection READ headDirection NOTIFY headDirectionChanged)
    Q_PROPERTY(QMatrix4x4 headOrientation READ headOrientation NOTIFY headOrientationChanged)
    Q_PROPERTY(bool running READ running NOTIFY runningChanged)
    Q_PROPERTY(int disc READ disc WRITE setDisc NOTIFY discChanged)
    Q_PROPERTY(float fov READ fov WRITE setFov NOTIFY fovChanged)

public:
    Renderdata();
    void connectReadInputWidthHeightFrom(Renderdata *other);
    void connectReadInputWidthHeightFrom(QQuickItem *other);
    void connectReadInputFrom(Renderdata *other);
    void connectWriteOutputTo(Renderdata *other);

    QmlEntitydata *entitydata() const;
    qreal width() const;
    qreal height() const;
    QMatrix4x4 matrix() const;
    bool vrmode() const;
    bool mirrorEnabled() const;
    bool mirrorDistorsion() const;
    bool mirrorRightEye() const;
    qreal pointSize() const;
    qreal distanceDetail() const;
    QString filename() const;
    QMatrix4x4 headMatrix() const;
    QVector3D headDirection() const;
    QMatrix4x4 headOrientation() const;
    bool running() const;
    int disc() const;
    float fov() const;

public Q_SLOTS:
    void setEntitydata(QmlEntitydata * entitydata);
    void setWidth(qreal width);
    void setHeight(qreal height);
    void setMatrix(QMatrix4x4 matrix);
    void setVrmode(bool vrmode);
    void setMirrorEnabled(bool mirrorEnabled);
    void setMirrorDistorsion(bool mirrorDistorsion);
    void setMirrorRightEye(bool mirrorRightEye);
    void setPointSize(qreal pointSize);
    void setDistanceDetail(qreal distanceDetail);
    void setFilename(QString filename);
    void setDisc(int disc);
    void setFov(float fov);

protected Q_SLOTS:
    void setHeadMatrix(QMatrix4x4 headMatrix);
    void setHeadDirection(QVector3D headDirection);
    void setHeadOrientation(QMatrix4x4 headOrientation);
    void setRunning(bool running);

    friend class RenderThread;
    friend class MapsRenderer;

Q_SIGNALS:
    void entitydataChanged(QmlEntitydata *entitydata);
    void widthChanged(qreal width);
    void heightChanged(qreal height);
    void matrixChanged(QMatrix4x4 matrix);
    void vrmodeChanged(bool vrmode);
    void mirrorEnabledChanged(bool mirrorEnabled);
    void mirrorDistorsionChanged(bool mirrorDistorsion);
    void mirrorRightEyeChanged(bool mirrorRightEye);
    void pointSizeChanged(qreal pointSize);
    void distanceDetailChanged(qreal distanceDetail);
    void filenameChanged(QString filename);
    void headMatrixChanged(QMatrix4x4 headMatrix);
    void headDirectionChanged(QVector3D headDirection);
    void headOrientationChanged(QMatrix4x4 headOrientation);
    void runningChanged(bool running);
    void discChanged(int disc);
    void fovChanged(float fov);

private:
    QmlEntitydata * m_entitydata;
    qreal m_width;
    qreal m_height;
    QMatrix4x4 m_matrix;
    bool m_vrmode;
    bool m_mirrorEnabled;
    bool m_mirrorDistorsion;
    bool m_mirrorRightEye;
    qreal m_pointSize;
    qreal m_distanceDetail;
    QString m_filename;
    QMatrix4x4 m_headMatrix;
    QVector3D m_headDirection;
    QMatrix4x4 m_headOrientation;
    bool m_running;
    int m_disc;
    float m_fov;
    std::shared_ptr<QMetaObject::Connection> m_connectionToEntitydata;

    void emitEntitiydataChanged(QmlEntitydata *entitydata);
};

#endif // MapsRenderer_H
