#ifndef QMLTREE
#define QMLTREE

#include <upns/typedefs.h>
#include <QtCore>
#include <mapit/msgs/services.pb.h>
#include <QQmlListProperty>

class QmlTree : public QObject
{
    Q_OBJECT

public:
    QmlTree(QObject *parent = nullptr);
    QmlTree(std::shared_ptr<mapit::msgs::Tree> &tree, QObject *parent = nullptr);

    Q_INVOKABLE QStringList getRefs();
    Q_INVOKABLE QString oidOfRef(QString name);

    Q_INVOKABLE bool isValid() const;
protected:
    std::shared_ptr<mapit::msgs::Tree> m_tree;
};

#endif
