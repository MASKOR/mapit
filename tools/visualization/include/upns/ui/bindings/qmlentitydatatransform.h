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
    QmlEntitydataTransform(std::shared_ptr<upns::AbstractEntitydata> &entitydata, QmlCheckout* co, QString path = "");
    std::shared_ptr<upns::AbstractEntitydata> getEntitydata() { return m_entitydata; }

    QMatrix4x4 matrix() const;

Q_SIGNALS:
    void matrixChanged(QMatrix4x4 matrix);

private Q_SLOTS:
    void setMatrix();
};

#endif
