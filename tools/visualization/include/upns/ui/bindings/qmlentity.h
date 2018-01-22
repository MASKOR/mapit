#ifndef QMLENTITY
#define QMLENTITY

#include <QtCore>
#include <upns/typedefs.h>
#include <mapit/msgs/services.pb.h>
#include <upns/ui/bindings/qmlstamp.h>

class QmlEntity : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString type READ type NOTIFY typeChanged)
    Q_PROPERTY(QString frameId READ frameId NOTIFY frameIdChanged)
    Q_PROPERTY(QmlStamp *stamp READ stamp NOTIFY stampChanged)

public:
    QmlEntity(QObject *parent = nullptr);
    QmlEntity(std::shared_ptr<mapit::msgs::Entity> &obj);

    Q_INVOKABLE bool isValid() const;
    QString type() const;
    QString frameId() const;
    QmlStamp *stamp();

Q_SIGNALS:
    void typeChanged(QString type); // To avoid error message of not notifyable property when using a property binding
    void frameIdChanged(QString type);
    void stampChanged(QmlStamp *stamp);

protected:
    std::shared_ptr<mapit::msgs::Entity> m_entity;
    QmlStamp *m_stamp;
};

#endif
