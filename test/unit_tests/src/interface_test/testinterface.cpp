#include "testinterface.h"

#include "../../src/autotest.h"

void TestInterface::init()
{
}

void TestInterface::cleanup()
{
}

void TestInterface::testGetMaps()
{
    QVERIFY(true);
    //QVERIFY(false);
}

void TestInterface::testGetLayer()
{
    QVERIFY(true);
    //QVERIFY(false);
}

DECLARE_TEST(TestInterface)
