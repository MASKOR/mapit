#include "testoperators.h"
#include "upns_globals.h"
#include "../../src/autotest.h"
#include <QDir>
#include <QVector>
#include <QString>
#include "yaml-cpp/yaml.h"

using namespace upns;

using Strings = upnsVec<upnsString>;
using Layers = upnsVec<Layer>;

Q_DECLARE_METATYPE( Strings );
Q_DECLARE_METATYPE( Layers );

void TestOperators::init()
{
}

void TestOperators::cleanup()
{
}

void TestOperators::initTestCase()
{
    const char* databaseName = "test.db";
    QDir dir(databaseName);
    if(dir.exists())
    {
        bool result = dir.removeRecursively();
        QVERIFY( result );
    }
    YAML::Node conf;
    YAML::Node mapsource;
    mapsource["name"] = "MapFileService";
    mapsource["filename"] = databaseName;
    conf["mapsource"] = mapsource;

    m_mapManager = new upns::MapManager(conf);
    m_mapService = m_mapManager->getInternalMapService();
}

void TestOperators::cleanupTestCase()
{
    delete m_mapService;
}

void TestOperators::testOperatorLoadPointcloud()
{
    OperationDescription desc;
    desc.set_operatorname("load_pointcloud");
    m_mapManager->doOperation( desc );
}

DECLARE_TEST(TestOperators)
