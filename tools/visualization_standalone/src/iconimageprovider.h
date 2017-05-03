#ifndef ICONIMAGEPROVIDER_H
#define ICONIMAGEPROVIDER_H
#include <QQuickImageProvider>

class IconImageProvider : public QQuickImageProvider
{
    QString m_iconFolder;
    bool m_iconSize;
public:
    IconImageProvider(QString iconFolder, bool iconSize = true);

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;
};

#endif
