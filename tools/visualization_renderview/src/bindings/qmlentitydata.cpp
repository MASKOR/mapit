#include "bindings/qmlentitydata.h"

QmlEntitydata::QmlEntitydata()
    :m_entitydata( NULL ),
     m_checkout( NULL ),
     m_path( "" )
{

}

QmlEntitydata::QmlEntitydata(upns::upnsSharedPointer<upns::AbstractEntityData> &entitydata, QmlCheckout *co, QString path)
    :m_entitydata(entitydata),
     m_checkout( co ),
     m_path( path )
{
    if(m_checkout)
    {
        connect(m_checkout, &QmlCheckout::intenalCheckoutChanged, this, &QmlEntitydata::setCheckout);
    }
}

QString QmlEntitydata::path() const
{
    return m_path;
}

QmlCheckout *QmlEntitydata::checkout() const
{
    return m_checkout;
}

void QmlEntitydata::setCheckout(QmlCheckout *checkout)
{
    if (m_checkout != checkout)
    {
        if(m_checkout)
        {
            disconnect(m_checkout, &QmlCheckout::intenalCheckoutChanged, this, &QmlEntitydata::setCheckout);
        }
        m_checkout = checkout;
        if(m_checkout)
        {
            connect(m_checkout, &QmlCheckout::intenalCheckoutChanged, this, &QmlEntitydata::setCheckout);
        }
        Q_EMIT checkoutChanged(checkout);
    }
    if( !m_path.isEmpty() && m_checkout->getCheckoutObj() )
    {
        m_entitydata = m_checkout->getCheckoutObj()->getEntitydataReadOnly(m_path.toStdString());
        Q_EMIT internalEntitydataChanged( this );
    }
}

void QmlEntitydata::setPath(QString path)
{
    if (m_path == path)
        return;

    m_path = path;
    if( m_checkout && m_checkout->getCheckoutObj() )
    {
        m_entitydata = m_checkout->getCheckoutObj()->getEntitydataReadOnly(m_path.toStdString());
        Q_EMIT internalEntitydataChanged( this );
    }
    Q_EMIT pathChanged(path);
}
