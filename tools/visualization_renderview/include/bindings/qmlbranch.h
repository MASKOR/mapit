#ifndef QMLBRANCH
#define QMLBRANCH

#include <QtCore>
#include <upns/typedefs.h>
#include <upns/services.pb.h>

class QmlBranch : public QObject
{
    Q_OBJECT
public:
    QmlBranch(QObject *parent = nullptr);
    QmlBranch(upns::upnsSharedPointer<upns::Branch> &branch );

protected:
    upns::upnsSharedPointer< upns::Branch > m_branch;
};

#endif
