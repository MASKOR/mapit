#ifndef QMLENTITY
#define QMLENTITY

#include <QtCore>
#include "upns.h"
#include "libs/upns_interface/services.pb.h"

class QmlEntity : public QObject
{
    Q_OBJECT
public:
    QmlEntity(upns::upnsSharedPointer<upns::Entity> &obj);

protected:
    upns::Entity m_entity;
};

#endif
