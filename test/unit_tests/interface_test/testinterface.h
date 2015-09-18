#ifndef __INTERFACETEST_H
#define __INTERFACETEST_H

#include <QTest>

class TestInterface : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void testGetMaps();
    void testGetLayer();


};

#endif
