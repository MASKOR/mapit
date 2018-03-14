/*******************************************************************************
 *
 * Copyright 2015-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "mapit/ui/bindings/qmlentity.h"
#include <mapit/typedefs.h>
#include <mapit/msgs/services.pb.h>

QmlEntity::QmlEntity(QObject *parent)
    : QObject(parent)
    , m_entity( nullptr )
    , m_stamp( nullptr )
{

}

QmlEntity::QmlEntity(std::shared_ptr<mapit::msgs::Entity> &obj)
    : m_entity( obj )
    , m_stamp( nullptr )
{

}


bool QmlEntity::isValid() const
{
    return m_entity != nullptr;
}

QString QmlEntity::type() const
{
    if(!m_entity) return "";
    return QString::fromStdString(m_entity->type());
}

QString QmlEntity::frameId() const
{
    if(!m_entity) return "";
    return QString::fromStdString(m_entity->frame_id());
}

QmlStamp *QmlEntity::stamp()
{
    if(!m_entity) return nullptr;
    if(!m_stamp)
    {
        m_stamp = new QmlStamp(m_entity->mutable_stamp(), this);
    }
    return m_stamp;
}
