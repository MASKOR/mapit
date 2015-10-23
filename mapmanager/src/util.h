#ifndef __UPNS_UTIL_H
#define __UPNS_UTIL_H

#include "upns_globals.h"
#include <string>
#include <leveldb/slice.h>
#include <boost/algorithm/string/replace.hpp>

namespace upns
{

    inline std::string idToString(const upnsuint64 &id);
    inline upnsuint64  stringToId(const char *id);
    inline upnsuint64  stringEndToId(const std::string &id);
    inline upnsuint64  sliceEndToId(const leveldb::Slice &id);
    inline upnsString  escapeName(const upnsString name);
    inline upnsString  unescapeName(const upnsString name);

    inline upnsuint64  generateId();


    inline std::string idToString(const upnsuint64 &id)
    {
        return std::string(reinterpret_cast<const char*>(&id), sizeof(upnsuint64));
    }

    inline upnsuint64 stringToId(const char* id)
    {
        return *reinterpret_cast<const upnsuint64*>(id);
    }

    inline upnsuint64 stringEndToId(const std::string &id)
    {
        const char* start = id.data() + (id.size() - sizeof(upnsuint64));
        return stringToId( start );
    }

    inline upnsuint64 sliceEndToId(const leveldb::Slice &id)
    {
        const char* start = id.data() + (id.size() - sizeof(upnsuint64));
        return stringToId( start );
    }

    inline upnsString escapeName(const upnsString name)
    {
        boost::replace_all_copy( name, "!", "\\!");
        return name;
    }

    inline upnsString unescapeName(const upnsString name)
    {
        boost::replace_all_copy( name, "\\!", "!");
        return name;
    }

    inline upnsuint64 generateId()
    {
        upnsuint64 ident;
        reinterpret_cast<upnsuint32*>(&ident)[0] = rand();
        reinterpret_cast<upnsuint32*>(&ident)[1] = rand();
        return ident;
    }

}
#endif
