/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2016 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef MAPIT_ERROR_H
#define MAPIT_ERROR_H

#include <algorithm>

#define MAPIT_STATUS_OK 0u

#define mapitIsOk(status) (status == MAPIT_STATUS_OK)

#define MAPIT_STATUS_ERROR 1u

#define MAPIT_STATUS_ERR_DB_NOT_FOUND 12u
#define MAPIT_STATUS_ERR_DB_CORRUPTION 13u
#define MAPIT_STATUS_ERR_DB_NOT_SUPPORTED 14u
#define MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT 15u
#define MAPIT_STATUS_ERR_DB_IO_ERROR 16u
#define MAPIT_STATUS_ERR_DB_UNKNOWN 17u

#define MAPIT_STATUS_ERR_DB_OPTIMISTIC_LOCKING 18u

#define MAPIT_STATUS_ERR_DB_PARSE 19u
#define MAPIT_STATUS_ERR_DB_DELETE_LAYER_FROM_MAP 20u

#define MAPIT_STATUS_ERR_MODULE_OPERATOR_NOT_FOUND 100u

#define MAPIT_STATUS_FILE_NOT_FOUND 101u
#define MAPIT_STATUS_INVALID_ARGUMENT 102u

#define MAPIT_STATUS_MAP_NOT_FOUND 103u
#define MAPIT_STATUS_LAYER_NOT_FOUND 104u
#define MAPIT_STATUS_ENTITY_NOT_FOUND 105u

#define MAPIT_STATUS_LAYER_TYPE_MISMATCH 106u

#define MAPIT_STATUS_REPOSITORY_NOT_EMPTY 107u

#define MAPIT_STATUS_BRANCH_ALREADY_EXISTS 108u

#define MAPIT_STATUS_INVALID_DATA 109u

#define MAPIT_STATUS_ERR_MODULE_TYPE_NOT_FOUND 110u

#define MAPIT_STATUS_ERR_NOT_YET_IMPLEMENTED 200u

#define MAPIT_STATUS_ERR_UNKNOWN 666u

#endif
