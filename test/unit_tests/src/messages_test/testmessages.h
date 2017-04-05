#ifndef __MESSAGESTEST_H
#define __MESSAGESTEST_H

#include <QTest>
#include <mapit/msgs/services.pb.h>

class TestMessages : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void testGetMaps();
    void testGetLayer();
};

#endif
