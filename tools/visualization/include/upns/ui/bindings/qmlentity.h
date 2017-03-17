#ifndef QMLENTITY
#define QMLENTITY

#include <QtCore>
#include <upns/typedefs.h>
#include <upns/services.pb.h>

class QmlEntity : public QObject
{
    Q_OBJECT
public:
    QmlEntity(QObject *parent = nullptr);
    QmlEntity(std::shared_ptr<upns::Entity> &obj);

    Q_INVOKABLE bool isValid() const;
protected:
    std::shared_ptr<upns::Entity> m_entity;
};

#endif
