#ifndef QMLTREE
#define QMLTREE

#include "upns.h"
#include <QtCore>
#include "libs/upns_interface/services.pb.h"
#include <QQmlListProperty>

class QmlTree : public QObject
{
    Q_OBJECT

public:
    QmlTree();
    QmlTree(upns::upnsSharedPointer<upns::Tree> &tree);

    Q_INVOKABLE QStringList getRefs();
    Q_INVOKABLE QString oidOfRef(QString name);

    Q_INVOKABLE bool isValid() const;
protected:
    upns::upnsSharedPointer<upns::Tree> m_tree;
};

#endif
