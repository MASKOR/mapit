#ifndef QMLENTITYDATA
#define QMLENTITYDATA

#include <QtCore>
#include <QJsonObject>
#include "libs/upns_interface/services.pb.h"
#include "abstractentitydata.h"

class QmlEntitydata : public QObject
{
    Q_OBJECT
public:
    QmlEntitydata();
    QmlEntitydata(upns::upnsSharedPointer<upns::AbstractEntityData> &entitydata);
    upns::upnsSharedPointer<upns::AbstractEntityData> getEntityData() { return m_entitydata; }
Q_SIGNALS:
    void updated();

protected:
    upns::upnsSharedPointer<upns::AbstractEntityData> m_entitydata;
};

#endif
