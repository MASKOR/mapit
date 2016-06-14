#include "models/qmlroottreemodel.h"

QmlRootTreeModel::QmlRootTreeModel()
    :m_root( NULL )
{

    m_roleNameMapping[Qt::DisplayRole] = "displayRole";
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

void QmlRootTreeModel::syncModel(QStandardItem *si, QmlTree *tr)
{
    if(si)
    {
        si->setData(QVariant::fromValue(tr));
    }
    QStringList children(tr->getRefs());
    for(QStringList::const_iterator iter(children.cbegin()); iter != children.cend(); ++iter)
    {
        QStandardItem *csi = new QStandardItem();
        csi->setData(*iter, Qt::DisplayRole);
        QString oid(tr->oidOfRef(*iter));
        QmlTree *tree = m_root->getTree(oid);
        if(tree == nullptr)
        {
            QmlEntity *ent = m_root->getEntity(oid);
            if(ent != nullptr)
            {
                csi->setData(QVariant::fromValue(ent));
            }
            //TODO: conflicts
        }
        else
        {
            syncModel(csi, tree);
        }
        if(si)
        {
            si->appendRow(csi);
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
