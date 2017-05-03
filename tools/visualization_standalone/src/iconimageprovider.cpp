#include "iconimageprovider.h"

IconImageProvider::IconImageProvider(QString iconFolder, bool iconSize)
    : QQuickImageProvider(QQuickImageProvider::Image)
    , m_iconFolder(iconFolder)
    , m_iconSize(iconSize)
{
    if(!m_iconFolder.endsWith("/"))
    {
        m_iconFolder += "/";
    }
}

QImage IconImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    QString sizeStr;
    if(m_iconSize)
    {
        if (requestedSize.width() == -1|| requestedSize.height() == -1)
        {
            sizeStr = "-24-ns.png";
            *size = QSize(24, 24);
        }
        else if (requestedSize.width() > 16 || requestedSize.height() > 16)
        {
            sizeStr = "-24-ns.png";
            *size = QSize(24, 24);
        }
        else
        {
            sizeStr = "-16-ns.png";
            *size = QSize(16, 16);
        }
    }

    QString resourceName(m_iconFolder + id + sizeStr);
    QImage img(resourceName);
    *size = img.size();
    return img;
}
