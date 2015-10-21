#ifndef __ABSTRACTLAYERDATALOADER_H
#define __ABSTRACTLAYERDATALOADER_H

#include "upns_globals.h"

namespace upns
{

/**
 * @brief The AbstractLayerDataStreamProvider class can be called by a concrete layerData implementation to store/read an abstract stream of data.
 * While AbstractLayerData is responsible for the communication from the mapmanager to the concrete implementation, this is the other direction.
 * Storage is a service implemented in a common way for all layerdata. A concrete Layerdata-Implementation must be able to "translate" between
 * streamed and localized data in 3D space. For native pointcloud2 this might be slow, as there is no information on where to find points of
 * a specific region in a 1 dimensional stream. For more sophisticated datastructures (octrees, ...) operations will be faster.
 */

class AbstractLayerDataStreamProvider
{
public:
    /**
     * @brief isCached indicates if reading operations are fast or if connection to storage device is slow (e.g. requests over network)
     * @return
     */
    bool isCached();

    /**
     * @brief startRead Used to read data from stream.
     * @param start offset in the stream. For slow connections the whole data may not be queried always.
     * @param len defaultvalue of 0 indicates, that data will be read until end
     * @return a stream to read data from
     */
    upnsIStream& startRead(upnsuint64 start = 0, upnsuint64 len = 0);
    void endRead(upnsIStream& strm);

    upnsOStream& startWrite(upnsuint64 start = 0, upnsuint64 len = 0);
    void endWrite(upnsOStream& strm);
};

}
#endif
