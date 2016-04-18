#ifndef TESTREPOSITORY_H
#define TESTREPOSITORY_H

#include <QTest>

#include "versioning/repository.h"

class TestRepository : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void testCreateCheckout();
    void testGetCheckout();
    void testCommit();

private:
    upns::Repository *m_repo;
};

#endif
