#include "qmlcommit.h"
#include <upns/typedefs.h>

QmlCommit::QmlCommit()
    :m_commit( nullptr )
{

}

QmlCommit::QmlCommit(upns::upnsSharedPointer<upns::Commit> &commit)
    :m_commit( commit )
{

}
