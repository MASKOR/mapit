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

#ifndef __UPNS_TYPEDEFS_H
#define __UPNS_TYPEDEFS_H

#include <cstddef> // for NULL, size_t, ...
#include <vector>
#include <string>
#include <iostream>
#include <memory>

namespace mapit {
  namespace msgs {
    class OperationDescription;
  }
}

namespace upns
{

typedef long long unsigned int upnsuint64;
typedef unsigned int upnsuint32;
typedef float upnsReal;
typedef std::istream upnsIStream;
typedef std::ostream upnsOStream;

typedef std::string CommitId;
typedef upnsuint64 MapIdentifier;
typedef upnsuint64 LayerIdentifier;
typedef upnsuint64 EntityIdentifier;

typedef upnsuint32 LockHandle;
typedef upnsuint32 StatusCode;

typedef std::string CommitId;
typedef std::string ObjectId;

// Paths to navigate through a checkout
// Path do not need to start with "/".
// TODO: There are not yet codepassages, where relative paths are needed
// Empty "directories" are not allowed ("//").
// Trailing "/" are omitted.
typedef std::string Path;

#define InvalidCommitId "invalidCId"
#define InvalidObjectId "invalidOId"

typedef std::pair<MapIdentifier, StatusCode> StatusPair;
typedef std::vector<StatusPair> MapResultsVector;

typedef std::pair<StatusCode, mapit::msgs::OperationDescription> OperationResult;
}

#endif // __UPNS_TYPEDEFS_H
