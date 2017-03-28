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
    QmlBranch(std::shared_ptr<upns::Branch> &branch );

protected:
    std::shared_ptr< upns::Branch > m_branch;
};

#endif
