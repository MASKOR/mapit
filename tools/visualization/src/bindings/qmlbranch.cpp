#include "upns/ui/bindings/qmlbranch.h"

QmlBranch::QmlBranch(QObject *parent)
    :QObject(parent), m_branch( nullptr )
{

}

QmlBranch::QmlBranch(std::shared_ptr<mapit::msgs::Branch> &branch)
    :m_branch( branch )
{

}
