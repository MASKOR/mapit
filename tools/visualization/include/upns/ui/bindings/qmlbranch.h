#ifndef QMLBRANCH
#define QMLBRANCH

#include <QtCore>
#include <upns/typedefs.h>
#include <mapit/msgs/services.pb.h>

class QmlBranch : public QObject
{
    Q_OBJECT
public:
    QmlBranch(QObject *parent = nullptr);
    QmlBranch(std::shared_ptr<mapit::msgs::Branch> &branch );

protected:
    std::shared_ptr< mapit::msgs::Branch > m_branch;
};

#endif
