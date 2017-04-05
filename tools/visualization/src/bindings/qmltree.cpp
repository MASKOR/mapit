#include "upns/ui/bindings/qmltree.h"

QmlTree::QmlTree(QObject *parent)
    :QObject(parent), m_tree( nullptr )
{

}

QmlTree::QmlTree(std::shared_ptr<mapit::msgs::Tree> &tree, QObject *parent)
    :QObject(parent), m_tree(tree)
{

}

QStringList QmlTree::getRefs()
{
    if(!m_tree) return QStringList();
    QStringList refs;
    for(google::protobuf::Map<std::string, mapit::msgs::ObjectReference >::const_iterator iter(m_tree->refs().cbegin());
        iter != m_tree->refs().cend();
        ++iter)
    {
        refs.append(QString::fromStdString(iter->first));
    }
    return refs;
}

QString QmlTree::oidOfRef(QString name)
{
    if(!m_tree) return "";
    return QString::fromStdString(m_tree->refs().at(name.toStdString()).id());
}

bool QmlTree::isValid() const
{
    return m_tree != nullptr;
}
