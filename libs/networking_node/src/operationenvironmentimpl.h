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

#ifndef __OPERATIONENVIRONMENTIMPL_H
#define __OPERATIONENVIRONMENTIMPL_H
#include <mapit/operators/operationenvironment.h>
#include <mapit/msgs/services.pb.h>

namespace mapit
{
namespace operators
{
class WorkspaceWritable;
}

class OperationEnvironmentImpl : public OperationEnvironment
{
public:
    OperationEnvironmentImpl(const mapit::msgs::OperationDescription& desc);
    void setWorkspace(operators::WorkspaceWritable *workspace);

    // OperationEnvironment Interface
    virtual operators::WorkspaceWritable *getWorkspace() const;
    virtual const mapit::msgs::OperationDescription *getDescription() const;
    virtual const std::string& getParameters() const;
    virtual void setOutputDescription(const std::string& out);
    virtual const mapit::msgs::OperationDescription outputDescription() const;

private:
    operators::WorkspaceWritable *m_workspace;
    const mapit::msgs::OperationDescription m_operationDesc;
    mapit::msgs::OperationDescription m_outDesc;
};

}

#endif
