#ifndef ICONIMAGEPROVIDER_H
#define ICONIMAGEPROVIDER_H
#include <QQuickImageProvider>

class IconImageProvider : public QQuickImageProvider
{
    QString m_iconFolder;
public:
    IconImageProvider(QString iconFolder);

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;
};

#endif
