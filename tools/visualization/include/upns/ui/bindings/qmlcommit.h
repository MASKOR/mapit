/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#ifndef QMLCOMMIT
#define QMLCOMMIT

#include <QtCore>
#include <upns/typedefs.h>
#include <mapit/msgs/services.pb.h>

class QmlCommit : public QObject
{
    Q_OBJECT
public:
    QmlCommit();
    QmlCommit(std::shared_ptr<mapit::msgs::Commit> &commit);
protected:
    std::shared_ptr< mapit::msgs::Commit > m_commit;
};

#endif
