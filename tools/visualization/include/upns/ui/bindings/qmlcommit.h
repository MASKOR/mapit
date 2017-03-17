#ifndef QMLCOMMIT
#define QMLCOMMIT

#include <QtCore>
#include <upns/typedefs.h>
#include <upns/services.pb.h>

class QmlCommit : public QObject
{
    Q_OBJECT
public:
    QmlCommit();
    QmlCommit(std::shared_ptr<upns::Commit> &commit);
protected:
    std::shared_ptr< upns::Commit > m_commit;
};

#endif
