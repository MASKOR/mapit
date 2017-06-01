#ifndef LASENTITYWRITER_H
#define LASENTITYWRITER_H

#include "liblas/liblas.hpp"
#include "upns/layertypes/lastype.h"
#include <upns/operators/serialization/abstractentitydataprovider.h>

class LASEntitydataWriterPrivate;
class LASEntitydataWriter /* acts like : public liblas::Writer */
{
public:
    ~LASEntitydataWriter();
    liblas::Header const& GetHeader() const;
//    void SetHeader(liblas::Header const& header);
    bool WritePoint(liblas::Point const& point);
//    void WriteHeader();
    void SetFilters(std::vector<liblas::FilterPtr> const& filters);
    std::vector<liblas::FilterPtr> GetFilters() const;
    void SetTransforms(std::vector<liblas::TransformPtr> const& transforms);
    std::vector<liblas::TransformPtr> GetTransforms() const;
private:
    LASEntitydataWriterPrivate *m_pimpl;
    LASEntitydataWriter(std::shared_ptr<upns::AbstractEntitydataProvider> prov, const liblas::Header& header);

    friend class LASEntitydata;
};

#endif
