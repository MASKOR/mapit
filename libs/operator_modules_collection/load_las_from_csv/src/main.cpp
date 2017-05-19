#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/lastype.h>
#include <upns/layertypes/lasentitydatawriter.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <iostream>
#include <memory>
#include <upns/errorcodes.h>
#include "liblas/liblas.hpp"
#include <fstream>

#include <QVector>
#include <QFile>
#include <QtMath>
#include <QJsonDocument>
#include <QJsonObject>

// Format:
// x,y,z,<anything>,...,...
// x,y,z,<anything>,...,...
class XyzCsvReader {
public:
    template<typename T>
    static qint64 readCsv(QString &path
                            , std::function<void(T* data, const int& i)> pointCallback
                            , std::function<void(const int &estimatedPoints)> reallocate
                            , int fields = 3)
    {
        QFile file(path);
        file.open(QFileDevice::ReadOnly);
        qint64 size = file.size();
        uchar *data = file.map(0, size);

        qint64 tokenBegin = 0;
        qint64 currentPoint = 0;
        int currentPointField = 0;
        T point[fields];
        int estimatedPoints = size/12/3; // assume mximum of 12 digits per float; 10 digits + 1 delim + 1 floatingpoint (+\n\r omitted)
        reallocate(estimatedPoints);
        for(qint64 idx = 0; idx < size ; ++idx)
        {
            if(  data[idx] != ','
                  && data[idx] != ';'
                  && data[idx] != '\n')
                continue;
            if(currentPointField < fields)
            {
                // token found and data needed, parse everything from last token to here
                bool ok;
                double d = QString(QByteArray(reinterpret_cast<const char*>(&data[tokenBegin]), idx-tokenBegin)).toDouble(&ok);
                if(ok)
                {
                    point[currentPointField] = d;
                    currentPointField++;
                }
                else
                {
                    // error, process point and skip until new line
                    for(int i=currentPointField ; i<fields ; ++i)
                    {
                        point[i] = 0.0;
                    }
                    currentPointField = fields;
                }
                if(currentPointField == fields)
                {
                    // point has enough data and can be processed
                    pointCallback(point, currentPoint);
                    currentPoint++;
                    currentPointField++; // nothing will happen until new line
                }
            }
            tokenBegin = idx + 1;
            if(data[idx] == '\n')
            {
                if(currentPointField < fields)
                {
                    //qDebug() << "incomplete point, not enough data";
                    // fill rest of point with 0.0
                    for(int i=currentPointField ; i<fields ; ++i)
                    {
                        point[i] = 0.0;
                    }
                    pointCallback(point, currentPoint);
                    currentPoint++;
                }
                if(estimatedPoints <= currentPoint)
                {
                    double signsPerPoint = idx/currentPoint;
                    qint64 signsPerPointInt = static_cast<qint64>(qFloor(signsPerPoint));
                    estimatedPoints = size/signsPerPointInt;//static_cast<int>(qCeil(estimatedMax));
                    log_info("Idx: " + std::to_string(idx) + ", p: " + std::to_string(currentPoint) + ", siz: " + std::to_string(size) + ", quot: " + std::to_string(signsPerPoint) + ", int: " + std::to_string(signsPerPointInt));
                    reallocate(estimatedPoints);
                }
                currentPointField = 0;
                if(idx+1 < size)
                {
                    if(data[idx+1] == '\r')
                    {
                        ++idx;
                        ++tokenBegin;
                    }
                }
            }
        }
        return currentPoint;
    }

    template<typename T, typename outT>
    static QVector<outT> readCsv(QVector<T> &doubles, QString &path
                                   , T *minx, T *miny, T *minz
                                   , T *maxx, T *maxy, T *maxz
                                   , T *offsetx, T *offsety, T *offsetz
                                   , int fields = 3
                                   , bool demean = true, bool flipYZ = false)
    {
        double max[fields];
        double min[fields];
        for(int i=0 ; i<fields ; ++i)
        {
            min[i] = +std::numeric_limits<double>::infinity();
            max[i] = -std::numeric_limits<double>::infinity();
        }
        qint64 pointsRead = readCsv<T>(path, [&](double *point, int idx)
        {
            for(int i=0 ; i<fields ; ++i)
            {
                if(min[i] > point[i]) min[i] = point[i];
                if(max[i] < point[i]) max[i] = point[i];
                doubles[idx + i] = point[i];
            }
        }, [&](int estimatedPoints)
        {
            doubles.resize(estimatedPoints*fields);
        }, fields);

        double ctr[fields];
        if(demean)
        {
            for(int i=0 ; i<fields ; ++i)
            {
                ctr[i] = min[i]*0.5+max[i]*0.5;
            }
        }
        else
        {
            for(int i=0 ; i<fields ; ++i)
            {
                ctr[i] = 0.0;
            }
        }
        QVector<outT> ret;
        ret.resize(pointsRead);
        const int maxDemeanFields = qMin(fields, 3);
        for(int i=0 ; i<pointsRead ; ++i)
        {
            for(int field=0 ; field<maxDemeanFields ; ++field)
            {
                ret[field] = doubles[i+field] - ctr[field];
            }
        }
        if(minx != nullptr) *minx = min[0];
        if(miny != nullptr) *miny = min[1];
        if(minz != nullptr) *minz = min[2];

        if(maxx != nullptr) *maxx = max[0];
        if(maxy != nullptr) *maxy = max[1];
        if(maxz != nullptr) *maxz = max[2];

        if(offsetx != nullptr) *offsetx = ctr[0];
        if(offsety != nullptr) *offsety = ctr[1];
        if(offsetz != nullptr) *offsetz = ctr[2];
        return ret;
    }
};





upns::StatusCode operate_load_las_csv(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();


    std::string filename = params["filename"].toString().toStdString();
    if(filename.empty())
    {
        log_error("parameter \"filename\" missing");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
// demean and normalize not supported!
// Reason: This would require to rebuild the read LAS datastructure and create custom points.
// Points may vary in dataformat which might result in multiple codepaths.
// Pcds can be demeaned.
//    bool demean = params["demean"].bool_value();
//    bool normalize = params["normalize"].bool_value();
//    double normalizeScale = params["normalizeScale"].number_value();
//    if(normalizeScale < 0.001)
//    {
//        if(normalizeScale < 0.0)
//        {
//            log_error("normalizeScale was negative");
//            return UPNS_STATUS_INVALID_ARGUMENT;
//        }
//        normalizeScale = 10.0; // 10m is default
//    }
//    else if(!normalize)
//    {
//        if(normalizeScale != 1.0)
//        {
//            log_warn("normalizeScale was set, but normalize was not active");
//        }
//        normalizeScale = 1.0;
//    }

//    std::ifstream ifs;
//    ifs.open(filename, std::ifstream::in);
//    if ( ! ifs.good() )
//    {
//        log_error("Couldn't read file" + filename);
//        return UPNS_STATUS_FILE_NOT_FOUND;
//    }

    std::shared_ptr<mapit::msgs::Entity> pclEntity(new mapit::msgs::Entity);
    pclEntity->set_type(LASEntitydata::TYPENAME());
    StatusCode s = env->getCheckout()->storeEntity(target, pclEntity);
    if(!upnsIsOk(s))
    {
        log_error("Failed to create entity.");
    }

    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );

    std::shared_ptr<LASEntitydata> entityData = std::static_pointer_cast<LASEntitydata>(abstractEntitydata);


    std::shared_ptr<liblas::Header> header(new liblas::Header());
    header->SetDataFormatId(liblas::ePointFormat1); // Time only

    // Set coordinate system using GDAL support
//    liblas::SpatialReference srs;
//    srs.SetFromUserInput("EPSG:4326"); //TODO

//    header.SetSRS(srs);
    header->SetCompressed(false);
    {
        std::unique_ptr<LASEntitydataWriter> writer = entityData->getWriter(header);
//    double minx, miny, minz
//         , maxx, maxy, maxz
//         , ctrx, ctry, ctrz;
//    QVector<double> points;
        QString file = QString::fromStdString(filename);
        //XyzCsvReader::readCsv<double, double>(points, file, &minx, &miny, &minz, &maxx, &maxy, &maxz, &ctrx, &ctry, &ctrz);
        int readPoints = XyzCsvReader::readCsv<double>(file, [&](double *point, int idx)
        {
            liblas::Point pointl(header.get());
            pointl.SetCoordinates(point[0], point[1], point[2]);
            // fill other properties of point record

            writer->WritePoint(pointl);
        }, [&](int estimatedPoints)
        {
            log_info("Estimated points in file: " + std::to_string(estimatedPoints));
        }, 3);
        log_info("Points read:" + std::to_string(readPoints));
        header->SetPointRecordsCount(readPoints);
        //header.SetRecordsCount(readPoints*3);
//        writer->SetHeader(header);
//        writer->WriteHeader();
    }
//    writer->SetFilters(reader.GetFilters());
//    writer->SetTransforms(reader.GetTransforms());
    //    std::copy(lasreader_iterator(reader),  lasreader_iterator(),
    //                      laswriter_iterator(writer));

//    int i=0;
//    if(demean || normalize)
//    {
//        double minX = reader.GetHeader().GetMinX();
//        double minY = reader.GetHeader().GetMinY();
//        double minZ = reader.GetHeader().GetMinZ();
//        double maxX = reader.GetHeader().GetMaxX();
//        double maxY = reader.GetHeader().GetMaxY();
//        double maxZ = reader.GetHeader().GetMaxZ();

//        double sx = maxX-minX;
//        double sy = maxY-minY;
//        double sz = maxZ-minZ;
//        double maxDim = std::max(std::max(sx, sy), sz);
//        double scale = normalizeScale / maxDim;

//        double cx = minX+sx*0.5;
//        double cy = minY+sy*0.5;
//        double cz = minZ+sz*0.5;

//        while(reader.ReadNextPoint())
//        {
//            liblas::Point current(reader.GetPoint());
//            liblas::Point newpoint(&reader.GetHeader());
//            newpoint.SetX((current.GetX()-cx)*scale);
//            newpoint.SetY((current.GetY()-cy)*scale);
//            newpoint.SetZ((current.GetZ()-cz)*scale);
//            writer->WritePoint(newpoint);
//            ++i;
//        }
//    }
//    else
//    {
//        while(reader.ReadNextPoint())
//        {
//            liblas::Point const&  current(reader.GetPoint());

//            writer->WritePoint(current);
//            ++i;
//        }
//    }
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Las File", "fhac", OPERATOR_VERSION, LASEntitydata_TYPENAME, &operate_load_las_csv)
