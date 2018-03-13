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
