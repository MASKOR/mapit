#include "qmlstubentitydatapointcloud2provider.h"
#include "layertypes_collection/pointcloud2/include/pointcloudlayer.h"
#include "serialization/entitystreammanager.h"
#include "abstractentitydata.h"

void QmlStubEntitydataPointcloud2Provider::setFilename(QString filename)
{
    if (m_filename == filename)
        return;

    m_filename = filename;
    Q_EMIT filenameChanged(filename);

    upnsSharedPointer<upns::AbstractEntityData> data = EntityStreamManager::getEntityDataImpl(&m_fileSerializer, "test.pcd", true, true);
    Q_EMIT updated();
}
