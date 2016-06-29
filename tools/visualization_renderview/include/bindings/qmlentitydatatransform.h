#ifndef QMLENTITYDATATRANSFORM
#define QMLENTITYDATATRANSFORM

#include <QObject>
#include <QMatrix4x4>
#include "qmlentitydata.h"
#include "qmlcheckout.h"

class QmlEntitydataTransform : public QmlEntitydata
{
    Q_OBJECT
    Q_PROPERTY(QMatrix4x4 matrix READ matrix NOTIFY matrixChanged)
public:
    QmlEntitydataTransform();
    QmlEntitydataTransform(upns::upnsSharedPointer<upns::AbstractEntityData> &entitydata, QmlCheckout* co, QString path = "");
    upns::upnsSharedPointer<upns::AbstractEntityData> getEntityData() { return m_entitydata; }

    QMatrix4x4 matrix() const;

public Q_SLOTS:

Q_SIGNALS:
    void matrixChanged(QMatrix4x4 matrix);
};

#endif
