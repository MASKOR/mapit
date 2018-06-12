/*******************************************************************************
 *
 * Copyright 2017-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "mapit/ui/bindings/qmltransform.h"
#include "mapit/ui/bindings/qmlworkspace.h"
#include <mapit/logging.h>
#include <mapit/layertypes/tflayer.h>
#include <mapit/layertypes/tflayer/tf2/buffer_core.h>
#include <mapit/layertypes/tflayer/tf2/exceptions.h>

QmlTransform::QmlTransform()
    : QObject()
    , m_mustExist(false)
    , m_workspace(nullptr)
    , m_path()
    , m_targetFrame()
    , m_sourceFrame()
    , m_stamp(new QmlStamp(nullptr, this))
{
    connect(this, &QmlTransform::updated, this, &QmlTransform::emitMatrixChanged);
    connect(this, &QmlTransform::updated, this, &QmlTransform::emitExistsChanged);
}

QMatrix4x4 QmlTransform::matrix() const
{
    if(!workspace() || !workspace()->getWorkspaceObj() || path().isEmpty())
    {
        return QMatrix4x4();
    }

    bool found;
    mapit::tf::TransformStamped tfs(getTfs(&found));
    if(!found)
    {
        return QMatrix4x4();
    }

    QMatrix4x4 matRot;
    // deserialize Tf2 Quaternion to Qt. If this is wrong, change also load_tf (load_tf.qml)
    // also change live-preview in this case and test diffrent transforms.
    QQuaternion quat(QVector4D(tfs.transform.rotation.x(),
                               tfs.transform.rotation.y(),
                               tfs.transform.rotation.z(),
                               tfs.transform.rotation.w()));
    matRot.rotate(quat);
    QMatrix4x4 matTr;
    matTr.translate(tfs.transform.translation.x(),
                    tfs.transform.translation.y(),
                    tfs.transform.translation.z());
    return matTr * matRot;
}

bool QmlTransform::mustExist() const
{
    return m_mustExist;
}

bool QmlTransform::exists() const
{
    if(!QmlTransform::workspace() || !QmlTransform::workspace()->getWorkspaceObj() || path().isEmpty())
    {
        return false;
    }
    std::shared_ptr<mapit::Workspace> workspace = QmlTransform::workspace()->getWorkspaceObj();
    std::string p = path().toStdString();
    std::shared_ptr<mapit::msgs::Entity> e(workspace->getEntity(p));
    if(!e)
    {
        return false;
    }
    bool found;
    getTfs(&found);
    return found;
}

void QmlTransform::setMustExist(bool mustExist)
{
    if (m_mustExist == mustExist)
        return;

    m_mustExist = mustExist;
    Q_EMIT mustExistChanged(mustExist);
}

void QmlTransform::emitMatrixChanged()
{
    // When entity updated (see derived class), matrix change is emitted.
    QMatrix4x4 mat = matrix();
    Q_EMIT matrixChanged(mat);
}

void QmlTransform::emitExistsChanged()
{
    bool e = exists();
    Q_EMIT existsChanged(e);
}

mapit::tf::TransformStamped QmlTransform::getTfs(bool *found) const
{
    // not safe (workspace, etc. are not checked for null)
    // extract entities mapname
    mapit::tf::TransformStamped tfs;

    std::shared_ptr<mapit::tf2::BufferCore> buffer = std::shared_ptr<mapit::tf2::BufferCore>(new mapit::tf2::BufferCore(workspace()->getWorkspaceObj().get(), ""));

    try
    {
        long sec =  (stamp() && stamp()->getStamp()) ? stamp()->sec()  : 0;
        long nsec = (stamp() && stamp()->getStamp()) ? stamp()->nsec() : 0;
        if (sec == 0 && nsec == 0) throw mapit::tf2::TransformException("no time given");
        tfs = buffer->lookupTransform(targetFrame().toStdString(),
                                      sourceFrame().toStdString(),
                                      mapit::time::from_sec_and_nsec(sec, nsec));
        if(found) *found = true;
        return tfs;
    }
    catch(mapit::tf2::TransformException e)
    {
//        log_info("Could not lookup transform " << sourceFrame().toStdString() << " -> " << targetFrame().toStdString());
        if(found) *found = false;

        // use identity on error
        return mapit::tf::TransformStamped();
    }
}

QmlWorkspace *QmlTransform::workspace() const
{
    return m_workspace;
}

QString QmlTransform::path() const
{
    return m_path;
}

QString QmlTransform::targetFrame() const
{
    return m_targetFrame;
}

QString QmlTransform::sourceFrame() const
{
    return m_sourceFrame;
}

QmlStamp *QmlTransform::stamp() const
{
    return m_stamp;
}

void QmlTransform::setWorkspace(QmlWorkspace *workspace)
{
    if (m_workspace == workspace)
        return;

    m_workspace = workspace;
    Q_EMIT workspaceChanged(m_workspace);
    Q_EMIT updated();
}

void QmlTransform::setPath(QString path)
{
    if (m_path == path)
        return;

    m_path = path;
    Q_EMIT pathChanged(m_path);
    Q_EMIT updated();
}

void QmlTransform::setTargetFrame(QString targetFrame)
{
    if (m_targetFrame == targetFrame)
        return;

    m_targetFrame = targetFrame;
    Q_EMIT targetFrameChanged(m_targetFrame);
    Q_EMIT updated();
    QMatrix4x4 mat = matrix();
    Q_EMIT matrixChanged(mat);
}

void QmlTransform::setSourceFrame(QString sourceFrame)
{
    if (m_sourceFrame == sourceFrame)
        return;

    m_sourceFrame = sourceFrame;
    Q_EMIT sourceFrameChanged(m_sourceFrame);
    Q_EMIT updated();
    QMatrix4x4 mat = matrix();
    Q_EMIT matrixChanged(mat);
}

void QmlTransform::setStamp(QmlStamp *stamp)
{
    if (m_stamp == stamp)
        return;

    m_stamp = stamp;
    Q_EMIT stampChanged(m_stamp);
}
