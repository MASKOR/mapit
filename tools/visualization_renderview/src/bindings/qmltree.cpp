#include "qmltree.h"

QmlTree::QmlTree()
    :m_tree( nullptr )
{

}

QmlTree::QmlTree(upns::upnsSharedPointer<upns::Tree> &tree)
    :m_tree(tree)
{

}

QStringList QmlTree::getRefs()
{
    QStringList refs;
    for(google::protobuf::Map<std::string, upns::ObjectReference >::const_iterator iter(m_tree->refs().cbegin());
        iter != m_tree->refs().cend();
        ++iter)
    {
        refs.append(QString::fromStdString(iter->first));
    }
    return refs;
}

QString QmlTree::oidOfRef(QString name)
{
    return QString::fromStdString(m_tree->refs().at(name.toStdString()).id());
}
