/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#ifndef BOUNDINGBOXLAYER_H
#define BOUNDINGBOXLAYER_H

#include <upns/entitydata.h>
#include <upns/operators/serialization/abstractentitydataprovider.h>
#include <mapit/msgs/datastructs.pb.h>

using namespace upns;

// TODO: Is import case needed? (dynamic linking?)
#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif
namespace mapit
{
  namespace msgs {
    typedef std::shared_ptr<mapit::msgs::Boundingbox> BoundingboxPtr;
  }
}

extern "C"
{
MODULE_EXPORT void createEntitydata(std::shared_ptr<AbstractEntitydata> *out, std::shared_ptr<AbstractEntitydataProvider> streamProvider);
}

class BoundingboxEntitydata : public Entitydata<mapit::msgs::Boundingbox>
{
public:
    static const char* TYPENAME();

    BoundingboxEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider);

    const char*         type() const;
    bool                hasFixedGrid() const;
    bool                canSaveRegions() const;
    mapit::msgs::BoundingboxPtr      getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                bool clipMode,
                                int lod = 0);
    int                 setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                mapit::msgs::BoundingboxPtr &data,
                                int lod = 0);

    mapit::msgs::BoundingboxPtr      getData(int lod = 0);
    int                 setData(mapit::msgs::BoundingboxPtr &data, int lod = 0);

    void gridCellAt(upnsReal   x, upnsReal   y, upnsReal   z,
                    upnsReal &x1, upnsReal &y1, upnsReal &z1,
                    upnsReal &x2, upnsReal &y2, upnsReal &z2) const;

    int getEntityBoundingBox(upnsReal &x1, upnsReal &y1, upnsReal &z1,
                             upnsReal &x2, upnsReal &y2, upnsReal &z2);

    upnsIStream *startReadBytes(upnsuint64 start, upnsuint64 len);
    void endRead(upnsIStream *&strm);

    upnsOStream *startWriteBytes(upnsuint64 start, upnsuint64 len);
    void endWrite(upnsOStream *&strm);

    size_t size() const;
private:
    std::shared_ptr<AbstractEntitydataProvider> m_streamProvider;
    mapit::msgs::BoundingboxPtr m_aabb;  // Axis Aligned Bounding Box
};

#endif // BOUNDINGBOXLAYER_H
