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

#include "mapit/ui/bindings/qmlstamp.h"

QmlStamp::QmlStamp(mapit::msgs::Time* s, QObject *parent)
    : QObject(parent)
    , m_stamp(s)
{
}

int QmlStamp::sec() const
{
    if(!m_stamp) return 0;
    return m_stamp->sec();
}

int QmlStamp::nsec() const
{
    if(!m_stamp) return 0;
    return m_stamp->nsec();
}

QString QmlStamp::text() const
{
    return this->toString();
}

const mapit::msgs::Time* QmlStamp::getStamp() const
{
    return m_stamp;
}

QString QmlStamp::toString() const
{
    if(!m_stamp) return "";
    return QString::number(m_stamp->sec()) + "s, " + QString::number(m_stamp->nsec()) + "ns";
}
