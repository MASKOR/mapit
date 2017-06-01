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
    Q_PROPERTY(bool mustExist READ mustExist WRITE setMustExist NOTIFY mustExistChanged)
    Q_PROPERTY(bool exists READ exists NOTIFY existsChanged)

public:
    QmlEntitydataTransform();
    QmlEntitydataTransform(std::shared_ptr<upns::AbstractEntitydata> &entitydata, QmlCheckout* co, QString path = "");
    std::shared_ptr<upns::AbstractEntitydata> getEntitydata() { return m_entitydata; }

    QMatrix4x4 matrix() const;
    bool mustExist() const;

    bool exists() const;

public Q_SLOTS:
    void setMustExist(bool mustExist);

Q_SIGNALS:
    void matrixChanged(QMatrix4x4 matrix);
    void mustExistChanged(bool mustExist);

    void existsChanged(bool exists);

private Q_SLOTS:
    void emitMatrixChanged();
    void emitExistsChanged();
private:
    bool m_mustExist;
    bool m_exists;
};

#endif
