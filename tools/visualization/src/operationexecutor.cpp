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

#include "operationexecutor.h"
#include <mapit/errorcodes.h>
#include <mapit/versioning/workspace.h>

void OperationExecutor::run()
{
    if(m_workspace)
    {
        mapit::OperationResult s = m_workspace->doOperation(m_desc);
        Q_EMIT operationExecuted( s.first );
    }
    else
    {
        Q_EMIT operationExecuted( MAPIT_STATUS_ERROR );
    }
}

OperationExecutor::OperationExecutor(QObject *parent, std::shared_ptr<mapit::Workspace> workspace, mapit::msgs::OperationDescription desc)
    : QThread(parent)
    , m_workspace( workspace )
    , m_desc( desc )
{}

OperationExecutor::~OperationExecutor() {wait();}
