#ifndef __TESTENTITYDATA_H
#define __TESTENTITYDATA_H

#include <QTest>

#include "versioning/repository.h"
#include "../repositorycommon.h"

namespace upns {
class AbstractEntityData;
class AbstractMapSerializer;
class AbstractEntityDataStreamProvider;
}

class TestEntitydata : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void testCreateLayer_data();
    void testCreateLayer();
private:
    void createTestdata();
    upns::upnsSharedPointer<upns::AbstractMapSerializer> m_ed[2];
    upns::upnsSharedPointer<upns::AbstractEntityDataStreamProvider> m_edsp[2];

};

#endif
