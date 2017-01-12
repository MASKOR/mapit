#ifndef __UPNS_UTIL_H
#define __UPNS_UTIL_H

#include "upns_typedefs.h"
#include <string>
#include <algorithm>
#include <google/protobuf/repeated_field.h>

namespace upns
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
