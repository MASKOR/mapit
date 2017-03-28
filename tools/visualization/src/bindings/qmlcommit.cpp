#include "upns/ui/bindings/qmlcommit.h"
#include <upns/typedefs.h>

QmlCommit::QmlCommit()
    :m_commit( nullptr )
{

}

QmlCommit::QmlCommit(std::shared_ptr<upns::Commit> &commit)
    :m_commit( commit )
{

}
