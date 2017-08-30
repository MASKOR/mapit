#ifndef OperatorLoaderWorker_H
#define OperatorLoaderWorker_H

#include <QThread>
#include <QtCore/QJsonObject>
#include <QVariant>

class OperatorLoader : public QThread
{
    Q_OBJECT
    void run();
public:
    OperatorLoader(QObject * parent = 0);
    ~OperatorLoader();
Q_SIGNALS:
    void operatorsAdded(QList<QVariant> result);
};
#endif
