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

#ifndef LASENTITYWRITER_H
#define LASENTITYWRITER_H
#define NOMINMAX
#include "liblas/liblas.hpp"
#include "mapit/layertypes/lastype.h"
#include <mapit/operators/serialization/abstractentitydataprovider.h>

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
    LASEntitydataWriter(std::shared_ptr<mapit::AbstractEntitydataProvider> prov, const liblas::Header& header);

    friend class LASEntitydata;
};

#endif
