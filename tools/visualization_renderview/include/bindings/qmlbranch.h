#ifndef QMLBRANCH
#define QMLBRANCH

#include <QtCore>
#include "upns.h"
#include "libs/upns_interface/services.pb.h"

class QmlBranch : public QObject
{
    Q_OBJECT
public:

protected:
    upns::upnsSharedPointer< upns::Branch > m_branch;
};

#endif
