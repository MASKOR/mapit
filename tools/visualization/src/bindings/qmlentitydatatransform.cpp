#include "upns/ui/bindings/qmlentitydatatransform.h"
#include "upns/ui/bindings/qmlcheckout.h"
#include <upns/logging.h>
#include <upns/layertypes/tflayer.h>

QmlEntitydataTransform::QmlEntitydataTransform()
    :QmlEntitydata()
{
    connect(this, &QmlEntitydata::updated, this, &QmlEntitydataTransform::setMatrix);
}

QmlEntitydataTransform::QmlEntitydataTransform(std::shared_ptr<upns::AbstractEntitydata> &entitydata, QmlCheckout *co, QString path)
    :QmlEntitydata(entitydata, co, path)
{
    connect(this, &QmlEntitydata::updated, this, &QmlEntitydataTransform::setMatrix);
}

QMatrix4x4 QmlEntitydataTransform::matrix() const
{
    if(!QmlEntitydata::checkout() || !QmlEntitydata::checkout()->getCheckoutObj() || path().isEmpty())
    {
        return QMatrix4x4();
    }
    std::shared_ptr<upns::Checkout> co = QmlEntitydata::checkout()->getCheckoutObj();
    std::string p = path().toStdString();
    if(!co->getEntity(p))
    {
        return QMatrix4x4();
    }
    std::shared_ptr<upns::AbstractEntitydata> abstractEntitydata = co->getEntitydataReadOnly(p);
    if(abstractEntitydata == NULL)
    {
        if(m_mustExist)
        {
            log_error("Wrong path given in qml for transform entity");
        }
        return QMatrix4x4();
    }
    if(abstractEntitydata->type() != TfEntitydata::TYPENAME())
    {
        //if(m_mustExist)
        //{
            log_error("Wrong path given in qml for transform entity (not a transform)");
        //}
        return QMatrix4x4();
    }

    QMatrix4x4 ret( std::static_pointer_cast<TfEntitydata>( abstractEntitydata )->getData()->data());
    return ret;
}

bool QmlEntitydataTransform::mustExist() const
{
    return m_mustExist;
}

void QmlEntitydataTransform::setMustExist(bool mustExist)
{
    if (m_mustExist == mustExist)
        return;

    m_mustExist = mustExist;
    Q_EMIT mustExistChanged(mustExist);
}

void QmlEntitydataTransform::setMatrix()
{
    QMatrix4x4 mat = matrix();
    Q_EMIT matrixChanged(mat);
}
