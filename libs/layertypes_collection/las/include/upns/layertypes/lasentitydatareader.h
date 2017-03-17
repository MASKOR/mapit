#ifndef LASENTITYREADER_H
#define LASENTITYREADER_H

#include "liblas/liblas.hpp"
#include "upns/layertypes/lastype.h"

class LASEntitydataReaderPrivate;
class LASEntitydataReader /* acts like : public liblas::Reader */
{
public:
    liblas::Point const& GetPoint() const;
    bool ReadNextPoint();
    bool ReadPointAt(std::size_t n);
    void Reset();
    bool Seek(std::size_t n);
    liblas::Point const& operator[](std::size_t n);
    void SetFilters(std::vector<liblas::FilterPtr> const& filters);
    std::vector<liblas::FilterPtr> GetFilters() const;
    void SetTransforms(std::vector<liblas::TransformPtr> const& transforms);
    std::vector<liblas::TransformPtr> GetTransforms() const;
private:
    LASEntitydataReaderPrivate *m_pimpl;
    LASEntitydataReader(std::shared_ptr<upns::AbstractEntitydataProvider> prov);
    friend class LASEntitydata;
};

#endif
