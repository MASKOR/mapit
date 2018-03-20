/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#include "operationenvironmentimpl.h"
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>

namespace mapit
{

OperationEnvironmentImpl::OperationEnvironmentImpl(const mapit::msgs::OperationDescription &desc)
    :m_operationDesc( desc )
{
}

void OperationEnvironmentImpl::setWorkspace(operators::WorkspaceWritable *workspace)
{
    m_workspace = workspace;
}

operators::WorkspaceWritable *OperationEnvironmentImpl::getWorkspace() const
{
    return m_workspace;
}

const mapit::msgs::OperationDescription *OperationEnvironmentImpl::getDescription() const
{
    return &m_operationDesc;
}

const std::string& OperationEnvironmentImpl::getParameters() const
{
    return m_operationDesc.params();
}

void OperationEnvironmentImpl::setOutputDescription(const std::string& out)
{
    m_outDesc.set_params(out);
}

const mapit::msgs::OperationDescription OperationEnvironmentImpl::outputDescription() const
{
    return m_outDesc;
}

}
