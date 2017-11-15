#ifndef TFLAYER_H
#define TFLAYER_H

#include <upns/entitydata.h>
#include <upns/operators/serialization/abstractentitydataprovider.h>

#include <upns/layertypes/tflayer/utils.h>
//TODO: Rename file to tfentitydata.h/cpp
using namespace upns;

namespace upns {
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
MODULE_EXPORT void createEntitydata(std::shared_ptr<AbstractEntitydata> *out, std::shared_ptr<AbstractEntitydataProvider> streamProvider);
//MODULE_EXPORT void deleteEntitydata(std::shared_ptr<AbstractEntitydata> streamProvider);
}

class TfEntitydata : public Entitydata<tf::store::TransformStampedList>
{
public:
  static const char* TYPENAME();

  TfEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider);

  const char*      type() const;
  bool             hasFixedGrid() const;
  bool             canSaveRegions() const;

  int              setData(upnsReal x1, upnsReal y1, upnsReal z1,
                           upnsReal x2, upnsReal y2, upnsReal z2,
                           std::shared_ptr<tf::store::TransformStampedList> &data,
                           int lod = 0);
  int              setData(std::shared_ptr<tf::store::TransformStampedList> &data, int lod = 0);

  std::shared_ptr<tf::store::TransformStampedList> getData(upnsReal x1, upnsReal y1, upnsReal z1,
                           upnsReal x2, upnsReal y2, upnsReal z2,
                           bool clipMode,
                           int lod = 0);
  std::shared_ptr<tf::store::TransformStampedList> getData(int lod = 0);

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
  std::shared_ptr<tf::store::TransformStampedList> m_transforms;

};

#endif
