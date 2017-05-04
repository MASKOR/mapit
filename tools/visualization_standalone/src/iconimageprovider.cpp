#include "iconimageprovider.h"
#include <QFile>
#include <QPainter>
#include <QFontMetrics>

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
    if (m_iconSize)
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
    else
    {
        sizeStr = ".png";
    }

    QString resourceName(m_iconFolder + id + sizeStr);
    QFile imgFile(resourceName);
    if(imgFile.exists())
    {
        QImage img(resourceName);
        *size = img.size();
        return img;
    }
    else
    {
        *size = requestedSize;
        if(requestedSize.width() <= 0 || requestedSize.height() <= 0)
        {
            *size = QSize(64,64);
        }
        QImage image(*size, QImage::Format_ARGB32_Premultiplied);
        QPainter painter(&image);

        QRgb hash = 0;
        for (int i = 0; i < id.length(); i++)
        {
            hash = id[i].toLatin1() + ((hash << 5) - hash);
        }
        QColor backgroundColor = QColor::fromRgb(hash);
        backgroundColor = backgroundColor.darker();
        backgroundColor = QColor(backgroundColor.red(), backgroundColor.green(), backgroundColor.blue());
        painter.fillRect(image.rect(), backgroundColor);

        QString name(id.mid(id.lastIndexOf("/")));
        name = name.toUpper().remove(QRegExp("[AEIOU_.\\s]"));
        while(name.length() > 6)
        {
            for (int i = 2; i < name.length()-1; i++)
            {
                name.remove(i, 1);
            }
        }

        QFont font("Helveteca");
        font.setBold(true);
        QFontMetrics fm(font);
        int textWidth = fm.width(name);
        float scaleFactor = static_cast<float>(image.width())/static_cast<float>(textWidth);
        QTransform trScale;
        trScale.translate(image.width()/2, image.height()/2);
        trScale.scale(scaleFactor, scaleFactor);
        painter.setTransform(trScale);
        painter.setPen(QColor("white"));
        painter.drawText(QRect(QPoint(image.width()*-0.5, image.height()*-0.5), image.size()), Qt::AlignCenter | Qt::AlignVCenter, name);
        return image;
    }
}
