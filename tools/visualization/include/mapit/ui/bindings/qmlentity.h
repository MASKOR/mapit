/*******************************************************************************
 *
 * Copyright 2016-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef QMLENTITY
#define QMLENTITY

#include <QtCore>
#include <mapit/typedefs.h>
#include <mapit/msgs/services.pb.h>
#include <mapit/ui/bindings/qmlstamp.h>

class QmlEntity : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString type READ type NOTIFY typeChanged)
    Q_PROPERTY(QString frameId READ frameId NOTIFY frameIdChanged)
    Q_PROPERTY(QmlStamp *stamp READ stamp NOTIFY stampChanged)

public:
    QmlEntity(QObject *parent = nullptr);
    QmlEntity(std::shared_ptr<mapit::msgs::Entity> &obj);

    Q_INVOKABLE bool isValid() const;
    QString type() const;
    QString frameId() const;
    QmlStamp *stamp();

Q_SIGNALS:
    void typeChanged(QString type); // To avoid error message of not notifyable property when using a property binding
    void frameIdChanged(QString type);
    void stampChanged(QmlStamp *stamp);

protected:
    std::shared_ptr<mapit::msgs::Entity> m_entity;
    QmlStamp *m_stamp;
};

#endif
