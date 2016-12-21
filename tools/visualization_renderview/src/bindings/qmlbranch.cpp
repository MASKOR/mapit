#include "qmlbranch.h"

QmlBranch::QmlBranch(QObject *parent)
    :QObject(parent), m_branch( nullptr )
{

}

QmlBranch::QmlBranch(upns::upnsSharedPointer<upns::Branch> &branch)
    :m_branch( branch )
{

}
