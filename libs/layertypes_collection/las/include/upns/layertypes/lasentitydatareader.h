#ifndef LASENTITYREADER_H
#define LASENTITYREADER_H
#define NOMINMAX
#include "liblas/liblas.hpp"
#include "upns/layertypes/lastype.h"

class LASEntitydataReaderPrivate;
class LASEntitydataReader /* acts like : public liblas::Reader */
{
public:
    liblas::Header const& GetHeader() const;
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
    ///
    /// \brief getReaderRaw WARNING: This readers stream lives only as long as this Entitydata Reader.
    /// By using this method some flexibility is stolen from LASEntitydataReader for datamanegement.
    /// \return reader
    ///
    liblas::Reader *getReaderRaw();
private:
    LASEntitydataReaderPrivate *m_pimpl;
    LASEntitydataReader(std::shared_ptr<upns::AbstractEntitydataProvider> prov);
    friend class LASEntitydata;
};

#endif
