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

#ifndef QMLROOTTREEMODEL_H
#define QMLROOTTREEMODEL_H

#include <QStandardItemModel>
#include "../bindings/qmlcheckout.h"

class QmlRootTreeModel : public QStandardItemModel
{
    Q_OBJECT
    Q_PROPERTY(QmlCheckout *root READ root WRITE setRoot NOTIFY rootChanged)
    Q_ENUMS(NodeType RootTreeViewRoles)

public:
    enum NodeType {
        TreeNode,
        EntityNode
    };
    enum RootTreeViewRoles {
        NodeDisplayRole = Qt::DisplayRole,
        NodePathRole = Qt::ToolTipRole,
        NodeTypeRole = Qt::UserRole + 1,
        NodeNodeRole = Qt::UserRole,
        NodeVisibleRole = Qt::UserRole + 2,
        NodeVisualInfoRole =  Qt::UserRole + 3
    };

    QmlRootTreeModel();
    QmlCheckout* root() const;

    Q_INVOKABLE QVariantMap get(int idx) const;
public Q_SLOTS:
    void setRoot(QmlCheckout *root);

    void syncModel();
    void syncModel(QStandardItem *si, QmlTree *tr, QString fullPath = "");

    QHash<int, QByteArray> roleNames() const;
Q_SIGNALS:
    void rootChanged(QmlCheckout *root);
    void itemsChanged();

private:
    QmlCheckout *m_root;
    QHash<int, QByteArray> m_roleNameMapping;
};

#endif
