#ifndef QMLENTITYDATA
#define QMLENTITYDATA

#include <QObject>
#include "libs/upns_interface/services.pb.h"
#include "abstractentitydata.h"
#include "qmlcheckout.h"

class QmlCheckout;

class QmlEntitydata : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QmlCheckout* checkout READ checkout WRITE setCheckout NOTIFY checkoutChanged)
    Q_PROPERTY(QString path READ path WRITE setPath NOTIFY pathChanged)
public:
    QmlEntitydata(QObject *parent = nullptr);
    QmlEntitydata(upns::upnsSharedPointer<upns::AbstractEntitydata> &entitydata, QmlCheckout* co, QString path = "");
    upns::upnsSharedPointer<upns::AbstractEntitydata> getEntitydata() { return m_entitydata; }

    QString path() const;

    QmlCheckout* checkout() const;

public Q_SLOTS:
    void setCheckout(QmlCheckout* checkout);
    void setPath(QString path);

Q_SIGNALS:
    void updated();
    void checkoutChanged(QmlCheckout* checkout);
    void pathChanged(QString path);

    void internalEntitydataChanged(QmlEntitydata *ed);
protected:
    upns::upnsSharedPointer<upns::AbstractEntitydata> m_entitydata;

private:
    QmlCheckout* m_checkout;
    QString m_path;
};

#endif
