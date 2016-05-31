#include "models/qmlroottreemodel.h"

QmlCheckout *QmlRootTreeModel::root() const
{
    return m_root;
}

void QmlRootTreeModel::setRoot(QmlCheckout *root)
{
    if (m_root == root)
        return;

    m_root = root;
    Q_EMIT rootChanged(root);
}

void QmlRootTreeModel::syncModel()
{
    QmlTree *root = m_root->getRoot();
    QStandardItem currentItem;

}

void QmlRootTreeModel::syncModel(QStandardItem &si, QmlTree *tr)
{
    si.setData(QVariant::fromValue(tr));
    QStringList children(tr->getRefs());
    for(QStringList::const_iterator iter(children.cbegin()); iter != children.cend(); ++iter)
    {
        QStandardItem csi;
        csi.setData(*iter, Qt::DisplayRole);
        QString oid(tr->oidOfRef(*iter));
        QmlTree *tree = m_root->getTree(oid);
        if(tree == nullptr)
        {
            QmlEntity *ent = m_root->getEntity(oid);
            if(ent != nullptr)
            {
                csi.setData(QVariant::fromValue(ent));
            }
            //TODO: conflicts
        }
        else
        {
            syncModel(csi, tree);
        }
        si.appendRow(&csi);
    }

}
