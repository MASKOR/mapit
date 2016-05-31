#include "qmlbranch.h"

QmlBranch::QmlBranch()
    :m_branch( nullptr )
{

}

QmlBranch::QmlBranch(upns::upnsSharedPointer<upns::Branch> &branch)
    :m_branch( branch )
{

}
