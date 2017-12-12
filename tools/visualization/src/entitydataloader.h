#ifndef ENTITYDATALOADER_H
#define ENTITYDATALOADER_H

#include <QThread>
#include <QtCore/QJsonObject>
#include <QVariant>
#include <memory>

namespace upns {
    class Checkout;
}
class EntitydataLoader : public QThread
{
    Q_OBJECT
    void run();
public:
    //EntitydataLoader(QObject * parent, QString repository, QString checkoutname, QString path );
    // This constructor requires upns::CHeckout to be thread-safe
    EntitydataLoader(QObject * parent, std::shared_ptr<upns::Checkout> co, QString path );
    ~EntitydataLoader();

Q_SIGNALS:
    void entityInfoLoaded(QJsonObject result);

private:
    QString m_repository;
    QString m_checkoutname;
    QString m_path;
    std::shared_ptr<upns::Checkout> m_checkout;
};

#endif
