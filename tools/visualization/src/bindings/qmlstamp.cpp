#include "upns/ui/bindings/qmlstamp.h"

QmlStamp::QmlStamp(mapit::msgs::Time* s, QObject *parent)
    : QObject(parent)
    , m_stamp(s)
{
}

int QmlStamp::sec() const
{
    if(!m_stamp) return 0;
    return m_stamp->sec();
}

int QmlStamp::nsec() const
{
    if(!m_stamp) return 0;
    return m_stamp->nsec();
}

QString QmlStamp::text() const
{
    return this->toString();
}

const mapit::msgs::Time* QmlStamp::getStamp() const
{
    return m_stamp;
}

QString QmlStamp::toString() const
{
    if(!m_stamp) return "";
    return QString::number(m_stamp->sec()) + "s, " + QString::number(m_stamp->nsec()) + "ns";
}
