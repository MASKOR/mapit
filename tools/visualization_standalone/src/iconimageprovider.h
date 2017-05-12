#ifndef ICONIMAGEPROVIDER_H
#define ICONIMAGEPROVIDER_H
#include <QQuickImageProvider>

class IconImageProvider : public QObject, public QQuickImageProvider
{
    Q_OBJECT
    Q_PROPERTY(bool darkIcons READ darkIcons WRITE setDarkIcons NOTIFY darkIconsChanged)
public:
    IconImageProvider(QString iconFolder, bool iconSize = true, bool generateMissing = false);

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;
    bool darkIcons() const;

public Q_SLOTS:
    void setDarkIcons(bool darkIcons);

Q_SIGNALS:
    void darkIconsChanged(bool darkIcons);

private:
    QString m_iconFolder;
    bool m_iconSize;
    bool m_darkIcons;
    bool m_generateMissing;
};

#endif
