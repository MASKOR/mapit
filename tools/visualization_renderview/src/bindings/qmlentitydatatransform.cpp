#include "bindings/qmlentitydatatransform.h"
#include "qmlcheckout.h"
#include "libs/layertypes_collection/tf/include/tflayer.h"
QmlEntitydataTransform::QmlEntitydataTransform()
    :QmlEntitydata()
{
    connect(this, &QmlEntitydata::updated, this, &QmlEntitydataTransform::setMatrix);
}

QmlEntitydataTransform::QmlEntitydataTransform(upns::upnsSharedPointer<upns::AbstractEntityData> &entitydata, QmlCheckout *co, QString path)
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
    upns::upnsSharedPointer<upns::Checkout> co = QmlEntitydata::checkout()->getCheckoutObj();
    upnsString p = path().toStdString();
    if(!co->getEntity(p))
    {
        return QMatrix4x4();
    }
    upns::upnsSharedPointer<upns::AbstractEntityData> abstractEntityData = co->getEntitydataReadOnly(p);
    if(abstractEntityData == NULL)
    {
        log_error("Wrong path given in qml for transform entity");
        return QMatrix4x4();
    }
    upns::upnsSharedPointer<TfEntitydata> entityData = upns::static_pointer_cast<TfEntitydata>( abstractEntityData );
    return *entityData->getData();
}

void QmlEntitydataTransform::setMatrix()
{
    QMatrix4x4 mat = matrix();
    Q_EMIT matrixChanged(mat);
}
