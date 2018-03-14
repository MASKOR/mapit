/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LASENTITYREADER_H
#define LASENTITYREADER_H
#define NOMINMAX
#include "liblas/liblas.hpp"
#include "mapit/layertypes/lastype.h"

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
    LASEntitydataReader(std::shared_ptr<mapit::AbstractEntitydataProvider> prov);
    friend class LASEntitydata;
};

#endif
