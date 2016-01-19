#ifndef QMLENTITYDATASTREAMPROVIDER
#define QMLENTITYDATASTREAMPROVIDER

#include <QtCore>
#include <QJsonObject>
#include "libs/upns_interface/services.pb.h"
#include "abstractentitydata.h"

class QmlEntitydata : public QObject
{
    Q_OBJECT
public:
    QmlEntitydata();
    upns::upnsSharedPointer<upns::AbstractEntityData> getEntityData() { return m_entitydata; }
Q_SIGNALS:
    void updated(upns::upnsSharedPointer<upns::AbstractEntityData> entitydata);

protected:
    upns::upnsSharedPointer<upns::AbstractEntityData> m_entitydata;
};

#endif
