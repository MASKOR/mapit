#include "bindings/qmlentitydatatransform.h"
#include "qmlcheckout.h"
#include "libs/layertypes_collection/tf/include/tflayer.h"
QmlEntitydataTransform::QmlEntitydataTransform()
    :QmlEntitydata()
{

}

QmlEntitydataTransform::QmlEntitydataTransform(upns::upnsSharedPointer<upns::AbstractEntityData> &entitydata, QmlCheckout *co, QString path)
    :QmlEntitydata(entitydata, co, path)
{
}

QMatrix4x4 QmlEntitydataTransform::matrix() const
{
    upns::upnsSharedPointer<upns::AbstractEntityData> abstractEntityData = QmlEntitydata::checkout()->getCheckoutObj()->getEntitydataReadOnly(path().toStdString());
    if(abstractEntityData == NULL)
    {
        log_error("Wrong path given in qml for transform entity");
        return QMatrix4x4();
    }
    upns::upnsSharedPointer<TfEntitydata> entityData = upns::static_pointer_cast<TfEntitydata>( abstractEntityData );
    return *entityData->getData();
}
