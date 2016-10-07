#ifndef __ABSTRACTLAYERDATALOADER_H
#define __ABSTRACTLAYERDATALOADER_H

#include "upns_globals.h"

namespace upns
{

/**
 * @brief The AbstractEntityDataStreamProvider class can be called by a concrete layerData implementation to store/read an abstract stream of data.
 * While AbstractLayerData is responsible for the communication from the mapmanager to the concrete implementation, this is the other direction.
 * Storage is a service implemented in a common way for all layerdata. A concrete Layerdata-Implementation must be able to "translate" between
 * streamed and localized data in 3D space. For native pointcloud2 this might be slow, as there is no information on where to find points of
 * a specific region in a 1 dimensional stream. For more sophisticated datastructures (octrees, ...) operations will be faster.
 * TODO: Might be called AbstractObjectDataStreamProvider
 * Do not read the write stream and do not write the read stream.
 */

class AbstractEntityDataStreamProvider
{
public:
    /**
     * @brief isCached indicates if reading operations are fast or if connection to storage device is slow (e.g. requests over network)
     * @return
     */
    virtual bool isCached() = 0;

    /**
     * @brief isReadWriteSame indicates, if reading the object is save while writing to the same position.
     * The underlaying implementation may operates on two different memorychunks or on the same.
     * @return
     */
    virtual bool isReadWriteSame() = 0;
    /**
     * @brief startRead Used to read data from stream.
     * @param start offset in the stream. For slow connections the whole data may not be queried always.
     * @param len defaultvalue of 0 indicates, that data will be read until end
     * @return a stream to read data from
     */
    virtual upnsIStream *startRead(upnsuint64 start = 0, upnsuint64 len = 0) = 0;
    /**
     * @brief endRead Tell underlying implementation, that the stream is no longer needed in memory.
     * May unlock other operations on the stream.
     * @param strm The stream returned by startRead()
     */
    virtual void endRead(upnsIStream *strm) = 0;

    /**
     * @brief startWrite Write data to an entity
     * @param start offset in the stream. For slow connections the whole data may not be queried always.
     * @param len defaultvalue of 0 indicates, that data will be written until end
     * @return a stream to write data to
     */
    virtual upnsOStream *startWrite(upnsuint64 start = 0, upnsuint64 len = 0) = 0;
    /**
     * @brief endWrite Tell the underlying implementation, that write operation has finished.
     * This will cause a rehashing of the stream, the entity and all parents.
     * @param strm
     */
    virtual void endWrite(upnsOStream *strm) = 0;

    virtual upnsuint64 getStreamSize() const = 0;

    virtual void setStreamSize(upnsuint64) = 0;

    /**
     * @brief lock If multiple writes are done and subsequent rehashing is not wanted, lock will supress rehashing.
     * @return
     */
    virtual LockHandle lock() = 0;

    /**
     * @brief unlock after unlocking rehashing will be done, if there were writes after locking.
     */
    virtual void unlock(LockHandle) = 0;
};

}
#endif
