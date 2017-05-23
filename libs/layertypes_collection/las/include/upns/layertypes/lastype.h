#ifndef LAZTYPE_H
#define LAZTYPE_H

#include <memory>
#include <upns/entitydata.h>
#include <upns/operators/serialization/abstractentitydataprovider.h>
#include "liblas/liblas.hpp"

#include "lasentitydatareader.h"
#include "lasentitydatawriter.h"

using namespace upns;

// TODO: Is import case needed? (dynamic linking?)
#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

extern "C"
{
MODULE_EXPORT void createEntitydata(std::shared_ptr<AbstractEntitydata> *out, std::shared_ptr<AbstractEntitydataProvider> streamProvider);
}

class LASEntitydataPrivate;
class LASEntitydataReader;
class LASEntitydataWriter;

///
/// \brief The LASEntitydata class
/// Exposes Reader and Writer for LAS/LAZ data.
/// Duo to the intrfae of libLAS the interface of this Entitytype is a bit more comple than other types.
/// Instead of having easy getData/setData methods, single points can be queried directly from the data.
/// In order to be compatible with ${insert_upns_framework_name_here} you should call start(Reading|Writing)LAS before accessing the data
/// and the corresponding end-Method afterwards.
/// E.g. calling endWritingLAS while accessing a network stream will trigger the actual network connection.
/// These Methods are not reentrant or thread safe for a single instance. However it should be possible to read and write at the same time (TODO: This must be tested).
/// It should moreover be possible to write multiple entitydatas at the same time. (TODO: This must be tested when accessing same blob and isReadWriteSame)
///
class LASEntitydata : public AbstractEntitydata
{
public:
    static const char* TYPENAME();

    LASEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider);
    ~LASEntitydata();

    const char *type() const;
    bool        hasFixedGrid() const;
    bool        canSaveRegions() const;

    void gridCellAt(upnsReal x, upnsReal y, upnsReal z, upnsReal &x1, upnsReal &y1, upnsReal &z1, upnsReal &x2, upnsReal &y2, upnsReal &z2) const;

    std::unique_ptr<LASEntitydataReader> getReader();

    ///
    /// IMPORTANT NOTICE: Header must be fully filled here. E.g. number of point must be known in advance
    /// liblas seems to have this abstracted (with setHeader and write header methods) but this is not
    /// the case and will lead to broken las files
    /// \brief getWriter
    /// \param header
    /// \return
    ///
    std::unique_ptr<LASEntitydataWriter> getWriter(const liblas::Header &header);

    upnsIStream *startReadBytes(upnsuint64 start, upnsuint64 len);
    void endRead(upnsIStream *strm);

    upnsOStream *startWriteBytes(upnsuint64 start, upnsuint64 len);
    void endWrite(upnsOStream *strm);

    size_t size() const;

private:
    LASEntitydataPrivate *m_pimpl;
};
#endif
