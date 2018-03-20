/*******************************************************************************
 *
 * Copyright 2016-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mapit/ui/models/qmlroottreemodel.h"

QmlRootTreeModel::QmlRootTreeModel()
    :m_root( NULL )
{
    m_roleNameMapping[NodeDisplayRole] = "displayRole";
    m_roleNameMapping[NodePathRole] = "path";
    m_roleNameMapping[NodeTypeRole] = "type";
    m_roleNameMapping[NodeNodeRole] = "node";
    m_roleNameMapping[NodeVisibleRole] = "visible";
    m_roleNameMapping[NodeVisualInfoRole] = "visualInfo";
}

QVariantMap QmlRootTreeModel::get(int idx) const
{
    QVariantMap map;
    Q_FOREACH(int k, roleNames().keys()) {
        map[roleNames().value(k)] = data(index(idx, 0), k);
    }
    return map;
}

QmlWorkspace *QmlRootTreeModel::root() const
{
    return m_root;
}

void QmlRootTreeModel::setRoot(QmlWorkspace *root)
{
    if (m_root != root)
    {
        if(m_root)
        {
            disconnect(m_root, &QmlWorkspace::internalWorkspaceChanged, this, &QmlRootTreeModel::setRoot);
        }
        m_root = root;
        if(m_root)
        {
            connect(m_root, &QmlWorkspace::internalWorkspaceChanged, this, &QmlRootTreeModel::setRoot);
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
                //csi->setData({}, NodeVisualInfoRole);
                csi->setData(false, NodeVisibleRole);
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
    Q_EMIT itemsChanged();
}

QHash<int, QByteArray> QmlRootTreeModel::roleNames() const
{
    return m_roleNameMapping;
}
