/*******************************************************************************
 *
 * Copyright 2017-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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
#include <mapit/layertypes/assettype.h>
#include <mapit/layertypes/openvdblayer.h>
#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <iostream>
#include <sstream>
#include <memory>
#include <mapit/errorcodes.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include "tinyply.h"
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>

using namespace mapit::msgs;

void normalForTri(const uint32_t& i1, const uint32_t& i2, const uint32_t i3,
                  std::vector<float> &normalsVec,
                  std::vector<int> &adjacentFaces,
                  const openvdb::v4_0_2::math::Vec3<float> &pV1,
                  const openvdb::v4_0_2::math::Vec3<float> &pV2,
                  const openvdb::v4_0_2::math::Vec3<float> &pV3)
{
    float a[3], b[3], crossprod[3], invlength;
    //get the difference in position (two edges of the triangle)
    //vec3 a = p2.xyz - p1.xyz;
    //vec3 b = p2.xyz - p3.xyz;
    a[0] = pV1[0]-pV2[0];
    a[1] = pV1[1]-pV2[1];
    a[2] = pV1[2]-pV2[2];
    b[0] = pV2[0]-pV3[0];
    b[1] = pV2[1]-pV3[1];
    b[2] = pV2[2]-pV3[2];

    //cross
    crossprod[0] = a[1]*b[2]-a[2]*b[1];
    crossprod[1] = a[2]*b[0]-a[0]*b[2];
    crossprod[2] = a[0]*b[1]-a[1]*b[0];

    //normalize, switch side (ccw triangles)
    invlength = 1.f/std::max(0.00000001f,sqrtf(pow(crossprod[0],2.f)+pow(crossprod[1],2.f)+pow(crossprod[2],2.f)));
    crossprod[0] *= invlength;
    crossprod[1] *= invlength;
    crossprod[2] *= invlength;

    normalsVec[i1*3] += crossprod[0];
    normalsVec[i1*3+1] += crossprod[1];
    normalsVec[i1*3+2] += crossprod[2];
    normalsVec[i2*3] += crossprod[0];
    normalsVec[i2*3+1] += crossprod[1];
    normalsVec[i2*3+2] += crossprod[2];
    normalsVec[i3*3] += crossprod[0];
    normalsVec[i3*3+1] += crossprod[1];
    normalsVec[i3*3+2] += crossprod[2];
    adjacentFaces[i1]++;
    adjacentFaces[i2]++;
    adjacentFaces[i3]++;
}

void generateAiSceneWithTinyPly(std::unique_ptr<openvdb::tools::VolumeToMesh> mesher, std::shared_ptr<AssetEntitydata> output)
{
    unsigned int trianglecount = 0;
    for (openvdb::Index64 n = 0, N = mesher->polygonPoolListSize(); n < N; ++n)
    {
        openvdb::tools::PolygonPool& polygons = mesher->polygonPoolList()[n];
        trianglecount += polygons.numTriangles();
        trianglecount += polygons.numQuads()*2;
    }
    log_info("Triangles: " + std::to_string(trianglecount) + ".");

//    // Scopes for memory deletion, only buf will survive
//    std::string buf;
//    {
        std::vector<uint32_t> indicesBuf;
        indicesBuf.resize(3 * trianglecount);


        std::vector<float> normalsVec;
        std::vector<int> adjacentFaces;
        normalsVec.resize(mesher->pointListSize() * 3);
        adjacentFaces.resize(mesher->pointListSize());

        for(int i=0 ; i < adjacentFaces.size() ; ++i)
        {
            normalsVec[i*3] = 0.0f;
            normalsVec[i*3+1] = 0.0f;
            normalsVec[i*3+2] = 0.0f;
            adjacentFaces[i] = 0;
        }

        unsigned int currentIndex = 0;
        for (openvdb::Index64 n = 0, N = mesher->polygonPoolListSize(); n < N; ++n)
        {
            const openvdb::tools::PolygonPool& polygons = mesher->polygonPoolList()[n];
            for (openvdb::Index64 i = 0, I = polygons.numQuads(); i < I; ++i) {
                const openvdb::Vec4I& quad = polygons.quad(i);
                uint32_t *currentIdx = &indicesBuf[currentIndex * 3];
                currentIdx[0] = quad[0];
                currentIdx[1] = quad[2];
                currentIdx[2] = quad[1];
                currentIdx[3] = quad[0];
                currentIdx[4] = quad[3];
                currentIdx[5] = quad[2];
                currentIndex += 2;

                const openvdb::v4_0_2::math::Vec3<float> &pV1 = mesher->pointList()[currentIdx[0]];
                const openvdb::v4_0_2::math::Vec3<float> &pV2 = mesher->pointList()[currentIdx[1]];
                const openvdb::v4_0_2::math::Vec3<float> &pV3 = mesher->pointList()[currentIdx[2]];
                normalForTri(currentIdx[0], currentIdx[1], currentIdx[2], normalsVec, adjacentFaces, pV1, pV2, pV3 );
                const openvdb::v4_0_2::math::Vec3<float> &pV4 = mesher->pointList()[currentIdx[3]];
                const openvdb::v4_0_2::math::Vec3<float> &pV5 = mesher->pointList()[currentIdx[4]];
                const openvdb::v4_0_2::math::Vec3<float> &pV6 = mesher->pointList()[currentIdx[5]];
                normalForTri(currentIdx[3], currentIdx[4], currentIdx[5], normalsVec, adjacentFaces, pV4, pV5, pV6 );
            }
            for (openvdb::Index64 i = 0, I = polygons.numTriangles(); i < I; ++i) {
                const openvdb::Vec3I& tri = polygons.triangle(i);
                uint32_t *currentIdx = &indicesBuf[currentIndex * 3];
                currentIdx[0] = tri[0];
                currentIdx[1] = tri[2];
                currentIdx[2] = tri[1];
                currentIndex += 1;

                openvdb::v4_0_2::math::Vec3<float> &pV1 = mesher->pointList()[currentIdx[0]];
                openvdb::v4_0_2::math::Vec3<float> &pV2 = mesher->pointList()[currentIdx[1]];
                openvdb::v4_0_2::math::Vec3<float> &pV3 = mesher->pointList()[currentIdx[2]];
                normalForTri(currentIdx[0], currentIdx[1], currentIdx[2], normalsVec, adjacentFaces, pV1, pV2, pV3 );
            }
        }
        assert(currentIndex == trianglecount);

        for(int i=0 ; i<adjacentFaces.size() ; i++)
        {
            float invlength = 1.f/std::max(1,adjacentFaces[i]);
            normalsVec[i*3] *= invlength;
            normalsVec[i*3+1] *= invlength;
            normalsVec[i*3+2] *= invlength;
        }

//        {
            AssetPtr myFile(new AssetDataPair(tinyply::PlyFile(), nullptr));
            tinyply::PlyFile *ply(&myFile->first);
            std::vector<float> vertsVec(&mesher->pointList()[0][0], &mesher->pointList()[0][0]+mesher->pointListSize() * 3);
            ply->add_properties_to_element("vertex", { "x", "y", "z" }, vertsVec);
            ply->add_properties_to_element("vertex", { "nx", "ny", "nz" }, normalsVec);
            ply->add_properties_to_element("face", { "vertex_indices" }, indicesBuf, 3, tinyply::PlyProperty::Type::UINT32);
            {
                output->setData(myFile);
//                mesher.reset(); // hopefully free some memory here
//                buf = ostrstr.str();
            }
//        }
//    }
//    size_t size = buf.size();
//    log_info("Binary ply size: " + std::to_string(size) + ".");
//    Assimp::Importer importer;
//    importer.ReadFileFromMemory(buf.data(), size, 0, "ply");
//    if(const char* err = importer.GetErrorString())
//    {
//        log_error("ply reading error. Reason: " + err);
//    }
//    log_info("ply read to assimp scene");
}

// JSON:
// - detail: mesh detail from 0 (very exact) to 1 (very low poly count)
// - input: input openvdb
// - output: output asset
// - target: input and output at the same time
mapit::StatusCode operate_ovdbtomesh(mapit::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());


    std::string input =  params["input"].toString().toStdString();
    std::string output = params["output"].toString().toStdString();
    if(input.empty())
    {
        input = params["target"].toString().toStdString();
        if(input.empty())
        {
            log_error("no input specified");
            return MAPIT_STATUS_INVALID_ARGUMENT;
        }
    }
    if(output.empty())
    {
        output = params["target"].toString().toStdString();
        if(output.empty())
        {
            log_error("no output specified");
            return MAPIT_STATUS_INVALID_ARGUMENT;
        }
    }

    double detail = params["detail"].toDouble();

//    if(detail > 1.0)
//    {
//        log_error("detail out of range. set to 1");
//        detail = 1.0;
//    }

    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydataInput = env->getWorkspace()->getEntitydataReadOnly( input );
    if(!abstractEntitydataInput)
    {
        log_error("input does not exist or is not readable.");
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }
    std::shared_ptr<FloatGridEntitydata> entityDataInput = std::dynamic_pointer_cast<FloatGridEntitydata>( abstractEntitydataInput );
    if(entityDataInput == nullptr)
    {
        log_error("Wrong type");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    FloatGridPtr inputGrid = entityDataInput->getData();

    std::shared_ptr<Entity> ent = env->getWorkspace()->getEntity(output);
    if(ent)
    {
        log_info("Output asset already exists. overwrite");
    }

    std::shared_ptr<Entity> assetEntity(new Entity);
    assetEntity->set_type(AssetEntitydata::TYPENAME());
    mapit::StatusCode s = env->getWorkspace()->storeEntity(output, assetEntity);
    if(!mapitIsOk(s))
    {
        log_error("Failed to create entity.");
        return MAPIT_STATUS_ERR_DB_IO_ERROR;
    }

    /// Input validation finished

    openvdb::FloatGrid::ConstPtr levelSetGrid = openvdb::gridConstPtrCast<openvdb::FloatGrid>(inputGrid);
    //TODO: use detail
    std::unique_ptr<openvdb::tools::VolumeToMesh> mesher(new openvdb::tools::VolumeToMesh( levelSetGrid->getGridClass() == openvdb::GRID_LEVEL_SET ? 0.0 : 0.01, detail));// ///*dragon*/0.1 );
    (*mesher)(*levelSetGrid);

    const int vertexCount = mesher->pointListSize();
    log_info("Generated mesh with " + std::to_string(vertexCount) + " vertices.");

    if(vertexCount == 0)
    {
        return MAPIT_STATUS_ERR_DB_IO_ERROR;
    }

//    openvdb::FloatGrid::ConstPtr levelSetGrid = openvdb::gridConstPtrCast<openvdb::FloatGrid>(inputGrid);
//    openvdb::CoordBBox bbox;
//    if (levelSetGrid->tree().evalLeafBoundingBox(bbox))
//    {
//        openvdb::Coord dim(bbox.max() - bbox.min());
//        openvdb::FloatGrid::ConstAccessor accessor = levelSetGrid->getConstAccessor();

//        openvdb::Coord xyz;
//        qreal density[8];

//        std::vector<openvdb::Vec3d> points(30000);
//        std::vector<openvdb::Vec3d> normals(30000);

//        // for each leaf..
//        for (openvdb::FloatGrid::TreeType::LeafCIter iter = levelSetGrid->tree().cbeginLeaf(); iter; iter.next()) {

//            //if (wasInterrupted()) break;


//            // for each active voxel..
//            openvdb::FloatGrid::TreeType::LeafNodeType::ValueOnCIter it = iter.getLeaf()->cbeginValueOn();
//            for ( ; it; ++it) {
//                xyz = it.getCoord();
//                bool isLess = false, isMore = false;

//                // Sample values at each corner of the voxel
//                for (unsigned int d = 0; d < 8; ++d) {

//                    openvdb::Coord valueCoord(
//                        xyz.x() +  (d & 1),
//                        xyz.y() + ((d & 2) >> 1),
//                        xyz.z() + ((d & 4) >> 2));

//                    // inverse sign convention for level sets!
//                    density[d] = float(accessor.getValue(valueCoord));
//                    density[d] <= 0.0f ? isLess = true : isMore = true;
//                }
//                openvdb::math::ScaleTranslateMap map;// = *(levelSetGrid->transform().map<openvdb::math::ScaleTranslateMap>());

//                openvdb::math::Gradient< openvdb::math::ScaleTranslateMap, openvdb::math::CD_2ND> grad;
//                openvdb::math::internal::ReturnValue<openvdb::FloatGrid::ConstAccessor>::Vec3Type n = grad.result(map, accessor, xyz);

//                // If there is a crossing, surface this voxel
//                if (isLess && isMore) {
//                    points.push_back(openvdb::Vec3d(
//                            xyz.x() ,
//                            xyz.y() ,
//                            xyz.z() ));

////                    openvdb::Vec3d n;
////                    // x
////                    int xi = (int)(x + 0.5f);
////                    float xf = x + 0.5f - xi;
////                    float xd0 = get_density(xi - 1, (int)y, (int)z);
////                    float xd1 = get_density(xi, (int)y, (int)z);
////                    float xd2 = get_density(xi + 1, (int)y, (int)z);
////                    res[0] = (xd1 - xd0) * (1.0f - xf) + (xd2 - xd1) * xf; // lerp
////                    // y
////                    int yi = (int)(y + 0.5f);
////                    float yf = y + 0.5f - yi;
////                    float yd0 = get_density((int)x, yi - 1, (int)z);
////                    float yd1 = get_density((int)x, yi, (int)z);
////                    float yd2 = get_density((int)x, yi + 1, (int)z);
////                    res[1] = (yd1 - yd0) * (1.0f - yf) + (yd2 - yd1) * yf; // lerp
////                    // z
////                    int zi = (int)(z + 0.5f);
////                    float zf = z + 0.5f - zi;
////                    float zd0 = get_density((int)x, (int)y, zi - 1);
////                    float zd1 = get_density((int)x, (int)y, zi);
////                    float zd2 = get_density((int)x, (int)y, zi + 1);
////                    res[2] = (zd1 - zd0) * (1.0f - zf) + (zd2 - zd1) * zf; // lerp
//                    normals.push_back(n);
//                }
//            } // end active voxel traversal
//        } // end leaf traversal

//        //if (wasInterrupted()) return;

////            if (mAdaptivityThreshold > 1e-6) {
////                GU_PolyReduceParms parms;
////                parms.percentage =
////                    static_cast<float>(100.0 * (1.0 - std::min(mAdaptivityThreshold, 0.99f)));
////                parms.usepercent = 1;
////                tmpGeo.polyReduce(parms);
////            }

//        const int vertexCount = points.size();
//        qDebug() << "points:" << vertexCount;

//        m_vaPosition->m_buffer.bind(); //Buffers must be bound to do "allocate", "map", "unmap", ...
//        m_vaPosition->m_buffer.allocate( vertexCount * 3 * sizeof(GLfloat));
//        m_mesh->addVertexAttribute( m_vaPosition );
//        float *const positionsGpu = reinterpret_cast<float *const>(m_vaPosition->m_buffer.map(QOpenGLBuffer::WriteOnly));

//        m_vaNormal->m_buffer.bind();
//        m_vaNormal->m_buffer.allocate(vertexCount * 3 * sizeof(GLfloat));
//        m_mesh->addVertexAttribute( m_vaNormal );
//        GLfloat *normalsGpu = static_cast<GLfloat*>(m_vaNormal->m_buffer.map( QOpenGLBuffer::WriteOnly ));

//        // world space transform
//        std::vector<openvdb::Vec3d>::const_iterator iter(points.begin());
//        int idx = 0;
//        while( iter != points.end() )
//        {
//            openvdb::Vec3d wPos = levelSetGrid->indexToWorld(*iter);
//            positionsGpu[idx*3  ] = static_cast<float>(wPos.x());
//            positionsGpu[idx*3+1] = static_cast<float>(wPos.y());
//            positionsGpu[idx*3+2] = static_cast<float>(wPos.z());
//            normalsGpu[idx*3  ] = static_cast<float>(normals[idx].x());
//            normalsGpu[idx*3+1] = static_cast<float>(normals[idx].y());
//            normalsGpu[idx*3+2] = static_cast<float>(normals[idx].z());
//            idx++;
//            iter++;
//        }
//        m_vaPosition->m_buffer.bind();
//        m_vaPosition->m_buffer.unmap();
//        m_vaNormal->m_buffer.bind();
//        m_vaNormal->m_buffer.unmap();
//        m_mesh->setPointInput( true );
//        m_mesh->setVertexCount( vertexCount );
//        m_mesh->setPrimitiveCount( vertexCount );
//    }

    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydataOutput = env->getWorkspace()->getEntitydataForReadWrite( output );
    if(!abstractEntitydataOutput)
    {
        log_error("could not read output asset");
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }
    std::shared_ptr<AssetEntitydata> entityDataOutput = std::dynamic_pointer_cast<AssetEntitydata>( abstractEntitydataOutput );
    if(!entityDataOutput)
    {
        log_error("could not cast output to FloatGrid");
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }
    generateAiSceneWithTinyPly(std::move(mesher), entityDataOutput);

//    OperationDescription out;
//    out.set_operatorname(OPERATOR_NAME);
//    out.set_operatorversion(OPERATOR_VERSION);
//    env->setOutputDescription( out.SerializeAsString() );
    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "make a mesh out of a levelset openvdb and assimp", "fhac", OPERATOR_VERSION, FloatGridEntitydata_TYPENAME, true, &operate_ovdbtomesh)
