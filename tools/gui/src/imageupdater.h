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

#ifndef IMAGEUPDATER_H
#define IMAGEUPDATER_H

#include <QObject>
#include <QQmlApplicationEngine>

class ImageUpdater : public QObject
{
    Q_OBJECT
public:
    ImageUpdater(QObject *parent, QQmlApplicationEngine *engine, QObject *appStyle)
        : QObject(parent)
        , m_engine(engine)
        , m_appStyle(appStyle)
    {

    }

public Q_SLOTS:
    void updateAllImages()
    {
        const QMetaObject *appStyleMeta = m_appStyle->metaObject();
        int isDarkIndex = appStyleMeta->indexOfProperty("isDark");
        bool isDark = appStyleMeta->property(isDarkIndex).read(m_appStyle).toBool();

        QList<QObject *> allElems = m_engine->rootObjects().first()->findChildren<QObject *>();
        Q_FOREACH(QObject * elem, allElems)
        {
            const QMetaObject *meta = elem->metaObject()->superClass();
            if(QString("QQuickImageBase").compare(meta->className()) == 0)
            {
                int sourceIdx = meta->indexOfProperty("source");
                QMetaProperty srcProp = meta->property(sourceIdx);
                QVariant source = srcProp.read(elem);

//                QByteArray sig = QMetaObject::normalizedSignature("setSource(const QUrl &url)");
//                int setSourceIdx = meta->indexOfMethod(sig);
//                QMetaMethod setSource = meta->method(setSourceIdx);

                QString str = source.toString().replace("_black_", "");
                str = str.replace("_white_", "");
                if(!isDark)
                    str.append("_white_");
                else
                    str.append("_black_");
                srcProp.write(elem, str);

//                setSource.invoke(elem, Qt::QueuedConnection, Q_ARG(QString, ""));
//                setSource.invoke(elem, Qt::QueuedConnection, Q_ARG(QVariant, source));
            }
        }
    }

Q_SIGNALS:
    void darkIconsChanged(bool darkIcons);

private:
    QQmlApplicationEngine *m_engine;
    QObject *m_appStyle;
};

#endif
