#ifndef __TESTTIME_H
#define __TESTTIME_H

#include <QTest>

class TestTime : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void testTime();
private:
    void createTestdata();

};

#endif
