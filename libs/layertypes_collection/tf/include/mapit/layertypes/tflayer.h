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

#ifndef TFLAYER_H
#define TFLAYER_H

#include <mapit/entitydata.h>
#include <mapit/operators/serialization/abstractentitydataprovider.h>

#include <mapit/layertypes/tflayer/utils.h>
//TODO: Rename file to tfentitydata.h/cpp

namespace mapit {
  namespace tf {
    namespace store {
      class TransformStampedList;
    }
  }
}

// TODO: Is import case needed? (dynmic linking?)
#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

extern "C"
{
//Note: Not possible in MSVC/Windows. Breaks shared pointers exchange
//MODULE_EXPORT std::shared_ptr<AbstractEntitydata> createEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider);
MODULE_EXPORT void createEntitydata(std::shared_ptr<mapit::AbstractEntitydata> *out, std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider);
//MODULE_EXPORT void deleteEntitydata(std::shared_ptr<AbstractEntitydata> streamProvider);
}

class TfEntitydata : public mapit::Entitydata<mapit::tf::store::TransformStampedList>
{
public:
  static const char* TYPENAME();

  TfEntitydata(std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider);

  const char*      type() const;
  bool             hasFixedGrid() const;
  bool             canSaveRegions() const;

  int              setData(float x1, float y1, float z1,
                           float x2, float y2, float z2,
                           std::shared_ptr<mapit::tf::store::TransformStampedList> &data,
                           int lod = 0);
  int              setData(std::shared_ptr<mapit::tf::store::TransformStampedList> &data, int lod = 0);

  std::shared_ptr<mapit::tf::store::TransformStampedList> getData(float x1, float y1, float z1,
                           float x2, float y2, float z2,
                           bool clipMode,
                           int lod = 0);
  std::shared_ptr<mapit::tf::store::TransformStampedList> getData(int lod = 0);

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
  std::shared_ptr<mapit::tf::store::TransformStampedList> m_transforms;

};

#endif
