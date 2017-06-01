#include "upns/ui/bindings/qmlcommit.h"
#include <upns/typedefs.h>

QmlCommit::QmlCommit()
    :m_commit( nullptr )
{

}

QmlCommit::QmlCommit(std::shared_ptr<mapit::msgs::Commit> &commit)
    :m_commit( commit )
{

}
