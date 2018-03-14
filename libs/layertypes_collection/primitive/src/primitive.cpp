/*******************************************************************************
 *
 * Copyright 2015-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "mapit/layertypes/primitive.h"
#include <mapit/logging.h>

template <typename T>
class not_deleter {
public:
  void operator()(T* ptr){}
};

const char *PrimitiveEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

PrimitiveEntitydata::PrimitiveEntitydata(std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider)
    :m_streamProvider( streamProvider ),
     m_primitive( nullptr )
{
}

const char* PrimitiveEntitydata::type() const
{
    return PrimitiveEntitydata::TYPENAME();
}

bool PrimitiveEntitydata::hasFixedGrid() const
{
    return false;
}

bool PrimitiveEntitydata::canSaveRegions() const
{
    return false;
}

PrimitivePtr PrimitiveEntitydata::getData(float x1, float y1, float z1,
                                                float x2, float y2, float z2,
                                                bool clipMode,
                                                int lod)
{
    if(m_primitive == NULL)
    {
        m_primitive = PrimitivePtr(new mapit::msgs::Primitive);
        mapit::istream *in = m_streamProvider->startRead();
        {
            if(!m_primitive->ParseFromIstream(in))
            {
                log_warn("Could not read tranforms from stream. Proceeding with empty path");
            }
        }
        m_streamProvider->endRead(in);
    }
    return m_primitive;
}

int PrimitiveEntitydata::setData(float x1, float y1, float z1,
                                 float x2, float y2, float z2,
                                 PrimitivePtr &data,
                                 int lod)
{
    mapit::ostream *out = m_streamProvider->startWrite();
    {
        data->SerializePartialToOstream(out);
    }
    m_streamProvider->endWrite(out);
	return 0; //TODO: MSVC: What to return here?
}

PrimitivePtr PrimitiveEntitydata::getData(int lod)
{
    return getData(-std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                   false, lod);
}

int PrimitiveEntitydata::setData(PrimitivePtr &data, int lod)
{
    return setData(-std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                   data, lod);
}

void PrimitiveEntitydata::gridCellAt(float   x, float   y, float   z,
                                     float &x1, float &y1, float &z1,
                                     float &x2, float &y2, float &z2) const
{
    x1 = -std::numeric_limits<float>::infinity();
    y1 = -std::numeric_limits<float>::infinity();
    z1 = -std::numeric_limits<float>::infinity();
    x2 = +std::numeric_limits<float>::infinity();
    y2 = +std::numeric_limits<float>::infinity();
    z2 = +std::numeric_limits<float>::infinity();
}

int PrimitiveEntitydata::getEntityBoundingBox(float &x1, float &y1, float &z1,
                                              float &x2, float &y2, float &z2)
{
    float size = 1.0;
    x1 = -size;
    y1 = -size;
    z1 = -size;
    x2 =  size;
    y2 =  size;
    z2 =  size;
    return 0;
}

mapit::istream *PrimitiveEntitydata::startReadBytes(mapit::uint64_t start, mapit::uint64_t len)
{
    return m_streamProvider->startRead(start, len);
}

void PrimitiveEntitydata::endRead(mapit::istream *&strm)
{
    m_streamProvider->endRead(strm);
}

mapit::ostream *PrimitiveEntitydata::startWriteBytes(mapit::uint64_t start, mapit::uint64_t len)
{
    return m_streamProvider->startWrite(start, len);
}

void PrimitiveEntitydata::endWrite(mapit::ostream *&strm)
{
    m_streamProvider->endWrite(strm);
}

size_t PrimitiveEntitydata::size() const
{
    return m_streamProvider->getStreamSize();
}

// Win32 does not like anything but void pointers handled between libraries
// For Unix there would be a hack to use a "custom deleter" which is given to the library to clean up the created memory
// the common denominator is to build pointer with custom deleter in our main programm and just exchange void pointers and call delete when we are done
//std::shared_ptr<AbstractEntitydata> createEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
//void* createEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
void deleteEntitydataPrimitive(mapit::AbstractEntitydata *ld)
{
    PrimitiveEntitydata *p = dynamic_cast<PrimitiveEntitydata*>(ld);
    if(p)
    {
        delete p;
    }
    else
    {
        log_error("Wrong entitytype");
    }
}
void createEntitydata(std::shared_ptr<mapit::AbstractEntitydata> *out, std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider)
{
    *out = std::shared_ptr<mapit::AbstractEntitydata>(new PrimitiveEntitydata( streamProvider ), deleteEntitydataPrimitive);
}

