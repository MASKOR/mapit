/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2016 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef LAYERTYPE_PRIMITIVE_H
#define LAYERTYPE_PRIMITIVE_H

#include <mapit/entitydata.h>
#include <mapit/operators/serialization/abstractentitydataprovider.h>

// TODO: Is import case needed? (dynmic linking?)
#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

extern "C"
{
//Note: Not possible in MSVC/Windows. Breaks shared pointers exchange
//MODULE_EXPORT std::shared_ptr<mapit::AbstractEntitydata> createEntitydata(std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider);
MODULE_EXPORT void createEntitydata(std::shared_ptr<mapit::AbstractEntitydata> *out, std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider);
//MODULE_EXPORT void deleteEntitydata(std::shared_ptr<mapit::AbstractEntitydata> streamProvider);
}

typedef std::shared_ptr<mapit::msgs::Primitive> PrimitivePtr;

class PrimitiveEntitydata : public mapit::Entitydata<mapit::msgs::Primitive>
{
public:
    static const char* TYPENAME();

    PrimitiveEntitydata(std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider);

    const char*         type() const;
    bool                hasFixedGrid() const;
    bool                canSaveRegions() const;
    PrimitivePtr        getData(float x1, float y1, float z1,
                                float x2, float y2, float z2,
                                bool clipMode,
                                int lod = 0);
    int                 setData(float x1, float y1, float z1,
                                float x2, float y2, float z2,
                                PrimitivePtr &data,
                                int lod = 0);

    PrimitivePtr        getData(int lod = 0);
    int                 setData(PrimitivePtr &data, int lod = 0);

    void gridCellAt(float   x, float   y, float   z,
                    float &x1, float &y1, float &z1,
                    float &x2, float &y2, float &z2) const;

    int getEntityBoundingBox(float &x1, float &y1, float &z1,
                             float &x2, float &y2, float &z2);

    mapit::istream *startReadBytes(mapit::uint64_t start, mapit::uint64_t len);
    void endRead(mapit::istream *&strm);

    mapit::ostream *startWriteBytes(mapit::uint64_t start, mapit::uint64_t len);
    void endWrite(mapit::ostream *&strm);

    size_t size() const;

private:
    std::shared_ptr<mapit::AbstractEntitydataProvider> m_streamProvider;
    PrimitivePtr m_primitive;
};

#endif
