/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2015 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef __UPNS_UTIL_H
#define __UPNS_UTIL_H

#include <mapit/typedefs.h>
#include <string>
#include <algorithm>
#include <google/protobuf/repeated_field.h>

namespace mapit
{

    bool protobufContains(::google::protobuf::RepeatedPtrField<std::string> *field, const ::std::string &str);

    inline size_t indexOfLastUnescapedDelim(const std::string &key)
    {
        size_t idx = key.find_last_of('!');
        do {
            if(idx == 0) break;
            if(idx == std::string::npos) break; // not found
            if(key[idx-1] != '\\') break; // do not return last escaped delim
            idx = key.find_last_of('!', idx-2);
        } while(true);
        return idx;
    }
}
#endif
