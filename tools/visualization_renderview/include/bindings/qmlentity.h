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
    QmlEntity(upns::upnsSharedPointer<upns::Entity> &obj);

    Q_INVOKABLE bool isValid() const;
protected:
    upns::upnsSharedPointer<upns::Entity> m_entity;
};

#endif
