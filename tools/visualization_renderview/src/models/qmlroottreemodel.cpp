#include "models/qmlroottreemodel.h"

QmlRootTreeModel::QmlRootTreeModel()
    :m_root( NULL )
{

    m_roleNameMapping[Qt::DisplayRole] = "displayRole";
    m_roleNameMapping[Qt::ToolTipRole] = "path";
    m_roleNameMapping[NodeTypeRole] = "type";
    m_roleNameMapping[Qt::UserRole] = "node";
}

QmlCheckout *QmlRootTreeModel::root() const
{
    return m_root;
}

void QmlRootTreeModel::setRoot(QmlCheckout *root)
{
    if (m_root != root)
    {
        if(m_root)
        {
            disconnect(m_root, &QmlCheckout::intenalCheckoutChanged, this, &QmlRootTreeModel::setRoot);
        }
        m_root = root;
        if(m_root)
        {
            connect(m_root, &QmlCheckout::intenalCheckoutChanged, this, &QmlRootTreeModel::setRoot);
        }
        Q_EMIT rootChanged(root);
    }
    syncModel();
}

void QmlRootTreeModel::syncModel()
{
    this->clear();
    QmlTree *root = m_root->getRoot();
    syncModel(NULL, root);
}

void QmlRootTreeModel::syncModel(QStandardItem *si, QmlTree *tr, QString fullPath)
{
    if(si)
    {
        QVariant oldTree = si->data(Qt::UserRole);
        if(oldTree.value<QmlTree*>() != nullptr) oldTree.value<QmlTree*>()->deleteLater();
        si->setData(QVariant::fromValue(tr), Qt::UserRole);
    }
    QStringList children(tr->getRefs());
    for(QStringList::const_iterator iter(children.cbegin()); iter != children.cend(); ++iter)
    {
        QString childFullPath = fullPath + "/" + *iter;
        QStandardItem *csi = new QStandardItem();
        csi->setData(*iter, Qt::DisplayRole);
        csi->setData(childFullPath, Qt::ToolTipRole);
        //QString oid(tr->oidOfRef(*iter));
        //QmlTree *tree = m_root->getTree(oid); //Note: oid is sometimes empty if object is transient. Why? TODO: specify path/oid correctly!
        QmlTree *tree = m_root->getTree(childFullPath);
        if(tree != nullptr && tree->isValid())
        {
            csi->setData(QmlRootTreeModel::TreeNode, NodeTypeRole);
            syncModel(csi, tree, childFullPath);
        }
        else
        {
            QmlEntity *ent = m_root->getEntity(childFullPath); //TODO: oid would get entity by oid, not path
            if(ent != nullptr && ent->isValid())
            {
                csi->setData(QVariant::fromValue(ent), Qt::UserRole);
                csi->setData(QmlRootTreeModel::EntityNode, NodeTypeRole);
            }
            //TODO: conflicts
        }
        if(si)
        {
            si->appendRow( csi );
        }
        else
        {
            this->appendRow( csi );
        }
    }
}

QHash<int, QByteArray> QmlRootTreeModel::roleNames() const
{
    return m_roleNameMapping;
}
