#ifndef QMLENTITY
#define QMLENTITY

#include <QtCore>
#include <upns/typedefs.h>
#include <mapit/msgs/services.pb.h>

class QmlEntity : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString type READ type NOTIFY typeChanged)
    QString m_type;

public:
    QmlEntity(QObject *parent = nullptr);
    QmlEntity(std::shared_ptr<mapit::msgs::Entity> &obj);

    Q_INVOKABLE bool isValid() const;
    QString type() const;

Q_SIGNALS:
    void typeChanged(QString type); // To avoid error message of not notifyable property when using a property binding

protected:
    std::shared_ptr<mapit::msgs::Entity> m_entity;
};

#endif
