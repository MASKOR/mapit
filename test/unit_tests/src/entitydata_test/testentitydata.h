#ifndef __TESTENTITYDATA_H
#define __TESTENTITYDATA_H

#include <QTest>

#include "versioning/repository.h"

namespace upns {
class RepositoryServer;
}

//TODO: Put test setup in common base class!

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
    upns::Repository *m_repo[3];
    upns::RepositoryServer* m_srv;
    std::function<void()> m_serverCallback;

    upns::Checkout *m_checkout[3];
    void createTestdata();
};

#endif
