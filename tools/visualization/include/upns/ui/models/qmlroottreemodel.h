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
        NodeTypeRole = Qt::UserRole + 1
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

private:
    QmlCheckout *m_root;
    QHash<int, QByteArray> m_roleNameMapping;
};

#endif
