#include "upns/ui/bindings/qmltransform.h"
#include "upns/ui/bindings/qmlcheckout.h"
#include <upns/logging.h>
#include <upns/layertypes/tflayer.h>
#include <upns/layertypes/tflayer/tf2/buffer_core.h>
#include <upns/layertypes/tflayer/tf2/exceptions.h>

QmlTransform::QmlTransform()
    : QObject()
    , m_mustExist(false)
    , m_checkout(nullptr)
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
    if(!checkout() || !checkout()->getCheckoutObj() || path().isEmpty())
    {
        return QMatrix4x4();
    }

    bool found;
    ::tf::TransformStamped tfs(getTfs(&found));
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
    return matRot * matTr;
}

bool QmlTransform::mustExist() const
{
    return m_mustExist;
}

bool QmlTransform::exists() const
{
    if(!QmlTransform::checkout() || !QmlTransform::checkout()->getCheckoutObj() || path().isEmpty())
    {
        return false;
    }
    std::shared_ptr<upns::Checkout> co = QmlTransform::checkout()->getCheckoutObj();
    std::string p = path().toStdString();
    std::shared_ptr<mapit::msgs::Entity> e(co->getEntity(p));
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

tf::TransformStamped QmlTransform::getTfs(bool *found) const
{
    // not safe (checkout, etc. are not checked for null)
    // extract entities mapname
    ::tf::TransformStamped tfs;

    std::shared_ptr<mapit::tf2::BufferCore> buffer = std::shared_ptr<mapit::tf2::BufferCore>(new mapit::tf2::BufferCore(checkout()->getCheckoutObj().get(), ""));

    try
    {
        long sec =  (stamp() && stamp()->getStamp()) ? stamp()->getStamp()->sec()  : 0;
        long nsec = (stamp() && stamp()->getStamp()) ? stamp()->getStamp()->nsec() : 0;
        tfs = buffer->lookupTransform(targetFrame().toStdString(),
                                      sourceFrame().toStdString(),
                                      mapit::time::from_sec_and_nsec(sec, nsec));
        if(found) *found = true;
        return tfs;
    }
    catch(mapit::tf2::TransformException e)
    {
        log_info("Could not lookup transform " << sourceFrame().toStdString() << " -> " << targetFrame().toStdString());
        if(found) *found = false;

        // use identity on error
        return ::tf::TransformStamped();
    }
}

QmlCheckout *QmlTransform::checkout() const
{
    return m_checkout;
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

void QmlTransform::setCheckout(QmlCheckout *checkout)
{
    if (m_checkout == checkout)
        return;

    m_checkout = checkout;
    Q_EMIT checkoutChanged(m_checkout);
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
