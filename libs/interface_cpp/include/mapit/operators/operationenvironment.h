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

#ifndef MAPIT_OPERATIONENVIRONMENT_H
#define MAPIT_OPERATIONENVIRONMENT_H

#include <string>

namespace mapit
{
  namespace msgs {
    class OperationDescription;
    class OperationParameter;
  }
}

namespace mapit
{
class CheckoutRaw;

class OperationEnvironment
{
public:
    /**
     * @brief mapServiceVersioned
     * This might be able to do a snapshot before the operation. afterwards it can see, what the operation did change.
     * @return do not delete this checkout!
     */
    virtual CheckoutRaw *getCheckout() const = 0;
    virtual const mapit::msgs::OperationDescription *getDescription() const = 0;
    virtual const std::string& getParameters() const = 0;
    virtual void setOutputDescription(const std::string&) = 0;
    virtual const mapit::msgs::OperationDescription outputDescription() const = 0;
};

}
#endif
