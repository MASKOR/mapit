/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <mapit/operators/module.h>
#include <mapit/logging.h>
#include <mapit/layertypes/lastype.h>
#include <mapit/layertypes/lasentitydatawriter.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <iostream>
#include <memory>
#include <mapit/errorcodes.h>
#include "liblas/liblas.hpp"
#include <fstream>

#include <QVector>
#include <QFile>
#include <QtMath>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>

// Format:
// x,y,z,<anything>,...,...
// x,y,z,<anything>,...,...
class XyzCsvReader {
public:
    template<typename T>
    static qint64 readCsv(QString &path
                            , std::function<void(T* data, const int& i)> pointCallback
                            , std::function<void(const int &estimatedPoints)> reallocate
                            , std::function<void(T* data, const int& i, const int &field, bool& skipPoint)> handleNaN
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
            if(   data[idx] != ','
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
                    bool skipPointOut;
                    handleNaN(point, currentPoint, currentPointField, skipPointOut);
//                    for(int i=currentPointField ; i<fields ; ++i)
//                    {
//                        point[i] = std::numeric_limits<double>::quiet_NaN();
//                    }
                    if(!skipPointOut)
                    {
                        ok = true;
                    }
                    currentPointField = fields;
                }
                if(currentPointField == fields)
                {
                    // point has enough data and can be processed
                    if(ok)
                    {
                        pointCallback(point, currentPoint);
                        currentPoint++;
                    }
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
//                    for(int i=currentPointField ; i<fields ; ++i)
//                    {
//                        point[i] = 0.0;
//                    }
                    bool skipPointOut;
                    handleNaN(point, currentPoint, currentPointField, skipPointOut);
                    if(!skipPointOut)
                    {
                        pointCallback(point, currentPoint);
                        currentPoint++;
                    }
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

    template<typename T>
    static qint64 readCsv(QVector<T> &doubles, QString &path
                                   , T *minx, T *miny, T *minz
                                   , T *maxx, T *maxy, T *maxz
                                   , T *offsetx, T *offsety, T *offsetz
                                   , T *scalex, T *scaley, T *scalez
                                   , int fields = 3)
    {
        double maximum[fields];
        double minimum[fields];
        for(int i=0 ; i<fields ; ++i)
        {
            minimum[i] = +std::numeric_limits<double>::infinity();
            maximum[i] = -std::numeric_limits<double>::infinity();
        }
        qint64 pointsRead = readCsv<T>(path, [&minimum, &maximum, &fields, &doubles](double *point, const int& idx)
        {
            for(int i=0 ; i<fields ; ++i)
            {
                if(minimum[i] > point[i]) minimum[i] = point[i];
                if(maximum[i] < point[i]) maximum[i] = point[i];
                doubles[idx*3 + i] = point[i];
            }
        }, [&](int estimatedPoints)
        {
            doubles.resize(estimatedPoints*fields);
        }, [&](double *point, const int& i, const int &field, bool& skipPoint)
        {
            log_warn("Point: " + std::to_string(i) + ", f: " + std::to_string(field) + " was invalid");
            skipPoint = true;
        }, fields);
        doubles.resize(pointsRead*fields);
        double ctr[fields];
        double maxDim = 0;
        for(int i=0 ; i<fields ; ++i)
        {
            ctr[i] = minimum[i]*0.5+maximum[i]*0.5;
            maxDim = std::max(maxDim, maximum[i]-minimum[i]);
        }
        //double scale = maxDim/static_cast<double>(std::numeric_limits<uint32_t>::max());

        if(scalex != nullptr) *scalex = 0.0001;//(max[0]-min[0])/10000.0;//static_cast<double>(std::numeric_limits<uint32_t>::max());
        if(scaley != nullptr) *scaley = 0.0001;//(max[1]-min[1])/10000.0;//static_cast<double>(std::numeric_limits<uint32_t>::max());
        if(scalez != nullptr) *scalez = 0.0001;//(max[2]-min[2])/10000.0;//static_cast<double>(std::numeric_limits<uint32_t>::max());

        if(minx != nullptr) *minx = minimum[0];
        if(miny != nullptr) *miny = minimum[1];
        if(minz != nullptr) *minz = minimum[2];

        if(maxx != nullptr) *maxx = maximum[0];
        if(maxy != nullptr) *maxy = maximum[1];
        if(maxz != nullptr) *maxz = maximum[2];

        if(offsetx != nullptr) *offsetx = ctr[0];
        if(offsety != nullptr) *offsety = ctr[1];
        if(offsetz != nullptr) *offsetz = ctr[2];
        return pointsRead;
    }
};

mapit::StatusCode operate_load_las_csv(mapit::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();


    std::string filename = params["filename"].toString().toStdString();
    if(filename.empty())
    {
        log_error("parameter \"filename\" missing");
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }
    std::shared_ptr<mapit::msgs::Entity> pclEntity(new mapit::msgs::Entity);
    pclEntity->set_type(LASEntitydata::TYPENAME());
    mapit::StatusCode s = env->getWorkspace()->storeEntity(target, pclEntity);
    if(!mapitIsOk(s))
    {
        log_error("Failed to create entity.");
    }

    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = env->getWorkspace()->getEntitydataForReadWrite( target );

    std::shared_ptr<LASEntitydata> entityData = std::dynamic_pointer_cast<LASEntitydata>(abstractEntitydata);
    if(entityData == nullptr)
    {
        log_error("Wrong type");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    QString file = QString::fromStdString(filename);

    double min[3], max[3];
    QVector<double> doubles;
    double ctr[3];
    double scale[3];
    qint64 pointsRead = XyzCsvReader::readCsv<double>(doubles, file
                    , &min[0], &min[1], &min[2]
                    , &max[0], &max[1], &max[2]
                    , &ctr[0], &ctr[1], &ctr[2]
                    , &scale[0], &scale[1], &scale[2] );
    log_info("Points read:" + std::to_string(doubles.size()/3) );
    assert(pointsRead == doubles.size()/3);
    liblas::Header header;
    header.SetDataFormatId(liblas::ePointFormat0);
    header.SetSoftwareId("mapit");
    header.SetRecordsCount(0);
    header.SetCompressed(false);
    header.SetPointRecordsCount(doubles.size()/3);
    header.SetMin(min[0], min[1], min[2]);
    header.SetMax(max[0], max[1], max[2]);
    header.SetScale(scale[0], scale[1], scale[2]);
    header.SetOffset(ctr[0], ctr[1], ctr[2]);
        // Set coordinate system using GDAL support
    //    liblas::SpatialReference srs;
    //    srs.SetFromUserInput("EPSG:4326"); //TODO
    //    header.SetSRS(srs);
    std::unique_ptr<LASEntitydataWriter> writer = entityData->getWriter(header);
    for(int i=0 ; i<doubles.size() ; i+=3)
    {
        liblas::Point pointl(&header);
        pointl.SetCoordinates(doubles.data()[i],
                              doubles.data()[i+1],
                              doubles.data()[i+2]);
        writer->WritePoint(pointl);
    }
    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "Loads a Las File", "fhac", OPERATOR_VERSION, LASEntitydata_TYPENAME, false, &operate_load_las_csv)
