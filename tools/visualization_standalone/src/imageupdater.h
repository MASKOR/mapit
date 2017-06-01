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
