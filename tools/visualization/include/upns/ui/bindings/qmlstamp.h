#ifndef QMLSTAMP
#define QMLSTAMP

#include <QObject>
#include <mapit/msgs/services.pb.h>

//TODO: Add fromDate toDate Qml Methods. Stamps will be used in Qml, UI.
//TODO: make Stamp compatibl with Qml (sec and nsec are long, which is not available in Qml)
class QmlStamp : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int sec READ sec NOTIFY secChanged)
    Q_PROPERTY(int nsec READ nsec NOTIFY nsecChanged)
    Q_PROPERTY(QString text READ text NOTIFY textChanged)

public:
    QmlStamp(mapit::msgs::Time* s, QObject *parent = nullptr);

    int sec() const;
    int nsec() const;

    QString text() const;

    const mapit::msgs::Time* getStamp() const;

public Q_SLOTS:
    QString toString() const;

Q_SIGNALS:
    void secChanged(int sec);
    void nsecChanged(int nsec);

    void textChanged(QString text);

private:
    mapit::msgs::Time *m_stamp;
};

#endif
