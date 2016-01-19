#ifndef QMLENTITYDATASTREAMPROVIDER
#define QMLENTITYDATASTREAMPROVIDER

#include <QtCore>
#include <QJsonObject>
#include "libs/upns_interface/services.pb.h"
#include "abstractentitydata.h"

class QmlEntitydataStreamProvider : public QObject
{
    Q_OBJECT
public:

Q_SIGNALS:
    void updated();

private:
    upns::upnsSharedPointer<upns::AbstractEntityData> m_entityData;
};

#endif
