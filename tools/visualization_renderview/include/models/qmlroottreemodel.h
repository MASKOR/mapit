#ifndef QMLROOTTREEMODEL_H
#define QMLROOTTREEMODEL_H

#include <QStandardItemModel>
#include "../bindings/qmlcheckout.h"

class QmlRootTreeModel : public QStandardItemModel
{
    Q_OBJECT
    Q_PROPERTY(QmlCheckout *root READ root WRITE setRoot NOTIFY rootChanged)

public:
    QmlRootTreeModel();
    QmlCheckout* root() const;

public Q_SLOTS:
    void setRoot(QmlCheckout *root);

    void syncModel();
    void syncModel(QStandardItem *si, QmlTree *tr);

    QHash<int, QByteArray> roleNames() const;
Q_SIGNALS:
    void rootChanged(QmlCheckout *root);

private:
    QmlCheckout *m_root;
    QHash<int, QByteArray> m_roleNameMapping;
};

#endif
