#ifndef QMLCOMMIT
#define QMLCOMMIT

#include <QtCore>
#include <upns/typedefs.h>
#include <mapit/msgs/services.pb.h>

class QmlCommit : public QObject
{
    Q_OBJECT
public:
    QmlCommit();
    QmlCommit(std::shared_ptr<mapit::msgs::Commit> &commit);
protected:
    std::shared_ptr< mapit::msgs::Commit > m_commit;
};

#endif
