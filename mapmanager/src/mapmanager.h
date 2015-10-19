#ifndef __MAPMANAGER_H
#define __MAPMANAGER_H

#include "upns_globals.h"

namespace upns
{

using MapIdentifier = upnsuint64;
using LayerIdentifier = upnsuint64;

class MapManager
{
    upnsVec<MapIdentifier> getMaps();
};

}
#endif
