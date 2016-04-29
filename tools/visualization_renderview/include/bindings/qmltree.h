#ifndef QMLTREE
#define QMLTREE

#include <QtCore>
#include "libs/upns_interface/services.pb.h"

class QmlTree : public QObject
{
    Q_OBJECT
public:

protected:
    upns::Tree m_entity;
};

#endif
