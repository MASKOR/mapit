#ifndef QMLSTUBENTITYDATAPOINTCLOUD2PROVIDER
#define QMLSTUBENTITYDATAPOINTCLOUD2PROVIDER

#include <QtCore>
#include <QJsonObject>
#include "libs/upns_interface/services.pb.h"
#include "qmlentitydatastreamprovider.h"
#include "stubs/fileserializer.h"

class QmlStubEntitydataPointcloud2Provider : public QmlEntitydataStreamProvider
{
    Q_OBJECT
    Q_PROPERTY(QString filename READ filename WRITE setFilename NOTIFY filenameChanged)
public:
    QString filename() const
    {
        return m_filename;
    }

public Q_SLOTS:
    void setFilename(QString filename);

Q_SIGNALS:
    void filenameChanged(QString filename);

private:
    QString m_filename;
    upns::FileSerializer m_fileSerializer;
};

#endif
