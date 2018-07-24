/*******************************************************************************
 *
 * Copyright      2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#ifndef QMLSTAMP
#define QMLSTAMP

#include <QObject>
#include <mapit/msgs/services.pb.h>

//TODO: Add fromDate toDate Qml Methods. Stamps will be used in Qml, UI.
//TODO: make Stamp compatible with Qml (sec and nsec are long, which is not available in Qml)
class QmlStamp : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int sec READ sec NOTIFY secChanged)
    Q_PROPERTY(int nsec READ nsec NOTIFY nsecChanged)
    Q_PROPERTY(QString text READ text NOTIFY textChanged)

public:
    QmlStamp(mapit::msgs::Time* s, QObject *parent = nullptr);

    int sec() const;
    int nsec() const;

    QString text() const;

    const mapit::msgs::Time* getStamp() const;

public Q_SLOTS:
    QString toString() const;

Q_SIGNALS:
    void secChanged(int sec);
    void nsecChanged(int nsec);

    void textChanged(QString text);

private:
    mapit::msgs::Time* m_stamp;
};

#endif
