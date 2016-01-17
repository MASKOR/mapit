#ifndef UPNS_HASH_H
#define UPNS_HASH_H

#include "upns_globals.h"
#include "services.pb.h"
#include <google/protobuf/message.h>
#include <string>
#include <sstream>
#include <functional>

namespace upns {

template<typename T>
ObjectId hash(const T &t);


template<>
ObjectId hash(const ::google::protobuf::Message &t)
{
    std::hash<std::string> stdHash;
    std::stringstream strstr;
    strstr << stdHash(t.SerializeAsString());
    return strstr.str();
}
template<>
ObjectId hash(const Tree &t)
{
    size_t finalHash_temp_todo = 0;
    std::stringstream strstr;
    ::google::protobuf::Map< ::std::string, ::upns::ObjectReference >::const_iterator iter(t.refs().cbegin());
    int count = 0;
    while(iter != t.refs().cend())
    {
        finalHash_temp_todo ^= std::hash<std::string>()(iter->first)    << (count);
        //finalHash_temp_todo ^= hash(iter->second)                  << (count+1);
        iter++;
        count++;
    }
    strstr << finalHash_temp_todo;
    return strstr.str();
}

template<>
ObjectId hash(const ObjectReference &t)
{
    size_t finalHash_temp_todo = 0;
    std::stringstream strstr;
    finalHash_temp_todo ^= std::hash<std::string>()(t.meta());
    strstr << finalHash_temp_todo;
    return strstr.str();
}

template<>
ObjectId hash(const Commit &t)
{
    std::hash<std::string> stdHash;
    std::stringstream strstr;
    strstr << stdHash(t.SerializeAsString());
    return strstr.str();
}

}
#endif
