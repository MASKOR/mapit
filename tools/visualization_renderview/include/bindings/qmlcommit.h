#ifndef QMLCOMMIT
#define QMLCOMMIT

#include <QtCore>
#include "upns.h"
#include "libs/upns_interface/services.pb.h"

class QmlCommit : public QObject
{
    Q_OBJECT
public:

protected:
    upns::upnsSharedPointer< upns::Commit > m_commit;
};

#endif
