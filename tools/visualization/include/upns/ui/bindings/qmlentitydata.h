#ifndef QMLENTITYDATA
#define QMLENTITYDATA

#include <QObject>
#include <mapit/msgs/services.pb.h>
#include <upns/abstractentitydata.h>
#include "qmlcheckout.h"

class QmlCheckout;
class EntitydataLoader;
class QmlEntitydata : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QmlCheckout* checkout READ checkout WRITE setCheckout NOTIFY checkoutChanged)
    Q_PROPERTY(QString path READ path WRITE setPath NOTIFY pathChanged)
    Q_PROPERTY(QJsonObject info READ info NOTIFY infoChanged)
    Q_PROPERTY(bool isLoading READ isLoading NOTIFY isLoadingChanged)
public:
    QmlEntitydata(QObject *parent = nullptr);
    QmlEntitydata(std::shared_ptr<upns::AbstractEntitydata> &entitydata, QmlCheckout* co, QString path = "");
    ~QmlEntitydata();
    std::shared_ptr<upns::AbstractEntitydata> getEntitydata();

    QString path() const;

    QmlCheckout* checkout() const;

    void updateInfo();

    QJsonObject info() const;

    bool isLoading() const;

public Q_SLOTS:
    void setCheckout(QmlCheckout* checkout);
    void setPath(QString path);
    void setInfo(QJsonObject info);

Q_SIGNALS:
    void updated();
    void checkoutChanged(QmlCheckout* checkout);
    void pathChanged(QString path);

    void internalEntitydataChanged(QmlEntitydata *ed);
    void infoChanged(QJsonObject info);

    void isLoadingChanged(bool isLoading);

protected:
    std::shared_ptr<upns::AbstractEntitydata> m_entitydata;

private:
    QmlCheckout* m_checkout;
    QString m_path;
    QJsonObject m_info;
    bool m_isLoading;
    EntitydataLoader *m_edLoader;
};

#endif
