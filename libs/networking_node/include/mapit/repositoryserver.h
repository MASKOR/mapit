/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#ifndef REPOSITORYSERVER_H
#define REPOSITORYSERVER_H

namespace mapit {

/**
 * @brief The RepositoryServer class
 * Use RepositoryNetworkingFactory to obtain an instance of this class.
 */

class RepositoryServer
{
public:
    virtual ~RepositoryServer() {}

    /**
     * @brief handleRequest handles the next request and blocks until one is received or timeout is reached.
     * If milliseconds is 0, it returns immediatly if there is no request.
     * If milliseconds is set to -1, this will wait infinite.
     * In the latter case, it can only be stopped by an system interrupt (not recommended).
     */
    virtual void handleRequest(int milliseconds) = 0;
};

}

#endif
