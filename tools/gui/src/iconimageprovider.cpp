/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "iconimageprovider.h"
#include <QFile>
#include <QPainter>
#include <QFontMetrics>

IconImageProvider::IconImageProvider(QString iconFolder, bool iconSize, bool generateMissing)
    : QQuickImageProvider(QQuickImageProvider::Image)
    , m_iconFolder( iconFolder )
    , m_iconSize( iconSize )
    , m_darkIcons( true )
    , m_generateMissing( generateMissing )
{
    if(!m_iconFolder.endsWith("/"))
    {
        m_iconFolder += "/";
    }

}

QImage IconImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    QString sizeStr;
    QString idFinal = id;
    idFinal = idFinal.replace("_white_", "");
    bool isDark = idFinal.compare(id);
    if(!isDark)
        idFinal = idFinal.replace("_black_", "");
    m_darkIcons = isDark;
    if (m_iconSize)
    {
        if(m_iconFolder.contains("material"))
        {
            sizeStr = m_darkIcons ? "_black" : "_white";
            if (requestedSize.width() == -1|| requestedSize.height() == -1)
            {
                sizeStr += "_24dp_2x.png";
                *size = QSize(48, 48);
            }
            else if (requestedSize.width() > 24 || requestedSize.height() > 24)
            {
                sizeStr += "_24dp_2x.png";
                *size = QSize(48, 48);
            }
            else
            {
                sizeStr += "_24dp_1x.png";
                *size = QSize(24, 24);
            }
        }
        else
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
    }
    else
    {
        sizeStr = ".png";
    }

    QString resourceName(m_iconFolder + idFinal + sizeStr);
    QFile imgFile(resourceName);
    if (imgFile.exists())
    {
        QImage img(resourceName);
        if (requestedSize.width() != -1 && requestedSize.height() != -1)
            img = img.scaled(requestedSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        else if (requestedSize.width() != -1 && requestedSize.height() == -1)
            img = img.scaled(requestedSize.width(), img.height(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        else if (requestedSize.width() == -1 && requestedSize.height() != -1)
            img = img.scaled(img.width(), requestedSize.height(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
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
        if(m_generateMissing)
        {

            QRgb hash = 0;
            for (int i = 0; i < id.length(); i++)
            {
                hash = id[i].toLatin1() + ((hash << 5) - hash);
            }
            QColor backgroundColor = QColor::fromRgb(hash);
            backgroundColor = backgroundColor.darker();
            backgroundColor = QColor(backgroundColor.red(), backgroundColor.green(), backgroundColor.blue());
            painter.fillRect(image.rect(), backgroundColor);

            if(requestedSize.width() == -1 || requestedSize.width() >= 10 )
            {
                QString name(id.mid(id.lastIndexOf("/")));
                float sizeFactor = 1.0;
                if(requestedSize.width() >= 30)
                {
                    name = name.toUpper().remove(QRegExp("[AEIOU_.\\s]"));
                    while(name.length() > 6)
                    {
                        for (int i = 2; i < name.length()-1; i++)
                        {
                            name.remove(i, 1);
                        }
                    }
                }
                else
                {
                    name = name.toUpper().remove(QRegExp("\\B[\\w]|[._\\s\\W]"));
                    sizeFactor = 0.8;
                }

                QFont font("Helvetica");
                font.setBold(true);
                QFontMetrics fm(font);
                int textWidth = fm.width(name);
                float scaleFactor = static_cast<float>(image.width()*sizeFactor)/static_cast<float>(textWidth);
                QTransform trScale;
                trScale.translate(image.width()/2, image.height()/2);
                trScale.scale(scaleFactor, scaleFactor);
                painter.setTransform(trScale);
                painter.setPen(QColor("white"));
                painter.drawText(QRect(QPoint(image.width()*-0.5, image.height()*-0.5), image.size()), Qt::AlignCenter | Qt::AlignVCenter, name);
            }
        }
        else
        {
            painter.fillRect(image.rect(), QColor("red"));
            QFont font("Helvetica");
            font.setBold(true);
            painter.setPen(QColor("white"));
            painter.drawText(image.rect(), Qt::AlignCenter | Qt::AlignVCenter, "N/A");
        }
        return image;
    }
}

bool IconImageProvider::darkIcons() const
{
    return m_darkIcons;
}

void IconImageProvider::setDarkIcons(bool darkIcons)
{
    if (m_darkIcons == darkIcons)
        return;

    m_darkIcons = darkIcons;
    Q_EMIT darkIconsChanged(darkIcons);
}
