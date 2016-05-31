#ifndef QMLCOMMIT
#define QMLCOMMIT

#include <QtCore>
#include "upns.h"
#include "libs/upns_interface/services.pb.h"

class QmlCommit : public QObject
{
    Q_OBJECT
public:
    QmlCommit();
    QmlCommit(upns::upnsSharedPointer<upns::Commit> &commit);
protected:
    upns::upnsSharedPointer< upns::Commit > m_commit;
};

#endif
