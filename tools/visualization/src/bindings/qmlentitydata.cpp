#include "upns/ui/bindings/qmlentitydata.h"
#include "entitydataloader.h"

QmlEntitydata::QmlEntitydata(QObject *parent)
    :QObject(parent),
     m_entitydata( NULL ),
     m_checkout( NULL ),
     m_path( "" ),
     m_isLoading( false ),
     m_edLoader(nullptr)
{

}

QmlEntitydata::QmlEntitydata(std::shared_ptr<upns::AbstractEntitydata> &entitydata, QmlCheckout *co, QString path)
    :m_entitydata(entitydata),
     m_checkout( co ),
     m_path( path ),
     m_isLoading( false ),
     m_edLoader(nullptr)
{
    if(m_checkout)
    {
        connect(m_checkout, &QmlCheckout::internalCheckoutChanged, this, &QmlEntitydata::setCheckout);
    }
}

QmlEntitydata::~QmlEntitydata()
{
    if(m_edLoader) delete m_edLoader;
}

std::shared_ptr<upns::AbstractEntitydata> QmlEntitydata::getEntitydata() { return m_entitydata; }

QString QmlEntitydata::path() const
{
    return m_path;
}

QmlCheckout *QmlEntitydata::checkout() const
{
    return m_checkout;
}

void QmlEntitydata::updateInfo()
{
    m_info = QJsonObject();
    Q_EMIT infoChanged(m_info);
    if(m_entitydata == nullptr) return;
    m_isLoading = true;
    Q_EMIT isLoadingChanged(m_isLoading);
    if(m_edLoader) delete m_edLoader;
    m_edLoader = new EntitydataLoader(this, m_checkout->getCheckoutObj(), m_path);
    connect(m_edLoader, &EntitydataLoader::entityInfoLoaded, this, &QmlEntitydata::setInfo);
    m_edLoader->start();
}

QJsonObject QmlEntitydata::info() const
{
    return m_info;
}

bool QmlEntitydata::isLoading() const
{
    return m_isLoading;
}

void QmlEntitydata::setCheckout(QmlCheckout *checkout)
{
    bool changed = false;
    if (m_checkout != checkout)
    {
        if(m_checkout)
        {
            disconnect(m_checkout, &QmlCheckout::internalCheckoutChanged, this, &QmlEntitydata::setCheckout);
        }
        m_checkout = checkout;
        if(m_checkout)
        {
            connect(m_checkout, &QmlCheckout::internalCheckoutChanged, this, &QmlEntitydata::setCheckout);
        }
        Q_EMIT checkoutChanged(checkout);
        changed = true;
    }
    if( !m_path.isEmpty() && m_checkout->getCheckoutObj() )
    {
        m_entitydata = m_checkout->getCheckoutObj()->getEntitydataReadOnly(m_path.toStdString());
        Q_EMIT internalEntitydataChanged( this );

        changed = true;
    }
    if(changed)
    {
        //Note: this is wrong sequence. updateInfo should be called before internalEntitydataChanged and checkoutChanged (?).
        updateInfo();
        Q_EMIT updated();
    }
}

void QmlEntitydata::setPath(QString path)
{
    if (m_path == path)
        return;

    m_path = path;
    if( m_checkout && m_checkout->getCheckoutObj() && !m_path.isEmpty() )
    {
        std::shared_ptr<mapit::msgs::Entity> e = m_checkout->getCheckoutObj()->getEntity(m_path.toStdString());
        if(e)
        {
            m_entitydata = m_checkout->getCheckoutObj()->getEntitydataReadOnly(m_path.toStdString());
            Q_EMIT internalEntitydataChanged( this );
        }
    }
    updateInfo();
    Q_EMIT pathChanged(path);
    Q_EMIT updated();
}

void QmlEntitydata::setInfo(QJsonObject info)
{
    m_isLoading = false;
    m_info = info;
    Q_EMIT isLoadingChanged(m_isLoading);
    Q_EMIT infoChanged(info);
}
