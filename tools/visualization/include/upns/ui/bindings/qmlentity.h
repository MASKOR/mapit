#ifndef QMLENTITY
#define QMLENTITY

#include <QtCore>
#include <upns/typedefs.h>
#include <mapit/msgs/services.pb.h>

class QmlEntity : public QObject
{
    Q_OBJECT
public:
    QmlEntity(QObject *parent = nullptr);
    QmlEntity(std::shared_ptr<mapit::msgs::Entity> &obj);

    Q_INVOKABLE bool isValid() const;
protected:
    std::shared_ptr<mapit::msgs::Entity> m_entity;
};

#endif
