#ifndef __UPNS_UTIL_H
#define __UPNS_UTIL_H

#include "upns_globals.h"
#include <string>
#include <leveldb/slice.h>
#include <algorithm>

#define UPNS_ID_LEN 40
namespace upns
{

    inline std::string idToString(const upnsuint64 &id);
    inline ObjectId stringToId(const char *id);
    inline ObjectId stringEndToId(const std::string &id);
    inline ObjectId sliceEndToId(const leveldb::Slice &id);
    inline upnsString  escapeName(const upnsString &name);
    inline upnsString  unescapeName(const upnsString &name);

    inline upnsuint64  generateId();


    inline std::string idToString(const upnsuint64 &id)
    {
        return std::string(reinterpret_cast<const char*>(&id), sizeof(upnsuint64));
    }

    inline ObjectId stringToId(const char* id)
    {
        return ObjectId( id );
    }

    inline ObjectId stringEndToId(const std::string &id)
    {
        const char* start = id.data() + (id.size() - sizeof(char)*UPNS_ID_LEN);
        return stringToId( start );
    }

    inline ObjectId sliceEndToId(const leveldb::Slice &id)
    {
        const char* start = id.data() + (id.size() - sizeof(char)*UPNS_ID_LEN);
        return stringToId( start );
    }

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

    inline upnsString sliceEndToName(const leveldb::Slice &name)
    {
        size_t idx = indexOfLastUnescapedDelim(name.data());
        idx++;
        if(idx >= name.size() || idx == std::string::npos) return "";
        return upnsString( name.data() + idx+1 );
    }

    inline upnsString stringEndToName(const std::string &name)
    {
        size_t idx = indexOfLastUnescapedDelim(name);
        idx++;
        if(idx >= name.length() || idx == std::string::npos) return "";
        return upnsString( &(name[idx+1]) );
    }

    inline upnsString escapeName(const upnsString name)
    {
        //TODO not yet used
        //std::replace( name.begin(), name.end(), "!", "\\!");
        return name;
    }

    inline upnsString unescapeName(const upnsString &name)
    {
        //TODO not yet used
        //std::replace( name.begin(), name.end(), "\\!", "!");
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
