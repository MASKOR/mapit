#ifndef QMLENTITYDATA
#define QMLENTITYDATA

#include <QObject>
#include <mapit/msgs/services.pb.h>
#include <upns/abstractentitydata.h>
#include "qmlcheckout.h"

class QmlCheckout;

class QmlEntitydata : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QmlCheckout* checkout READ checkout WRITE setCheckout NOTIFY checkoutChanged)
    Q_PROPERTY(QString path READ path WRITE setPath NOTIFY pathChanged)
public:
    QmlEntitydata(QObject *parent = nullptr);
    QmlEntitydata(std::shared_ptr<upns::AbstractEntitydata> &entitydata, QmlCheckout* co, QString path = "");
    std::shared_ptr<upns::AbstractEntitydata> getEntitydata() { return m_entitydata; }

    QString path() const;

    QmlCheckout* checkout() const;

    Q_INVOKABLE QVariant getInfo(QString propertyName);

public Q_SLOTS:
    void setCheckout(QmlCheckout* checkout);
    void setPath(QString path);

Q_SIGNALS:
    void updated();
    void checkoutChanged(QmlCheckout* checkout);
    void pathChanged(QString path);

    void internalEntitydataChanged(QmlEntitydata *ed);
protected:
    std::shared_ptr<upns::AbstractEntitydata> m_entitydata;

private:
    QmlCheckout* m_checkout;
    QString m_path;
};

#endif
