#ifndef QMLENTITY
#define QMLENTITY

#include <QtCore>
#include "upns.h"
#include "libs/upns_interface/services.pb.h"

class QmlEntity : public QObject
{
    Q_OBJECT
public:
    QmlEntity();
    QmlEntity(upns::upnsSharedPointer<upns::Entity> &obj);

    Q_INVOKABLE bool isValid() const;
protected:
    upns::upnsSharedPointer<upns::Entity> m_entity;
};

#endif
