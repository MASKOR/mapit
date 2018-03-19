/*******************************************************************************
 *
 * Copyright 2016-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#ifndef TESTREPOSITORIESCOM_H
#define TESTREPOSITORIESCOM_H

#include <QTest>

#include <mapit/versioning/repository.h>

namespace mapit {
class RepositoryServer;
}

//NOTE: This does not use Repositorycommon because it tests special network cases. RepositoryCommon uses standard network setups.
// This class is hard to debug because of fork(). Other Testclasses use proper threads.

class TestRepositoriesCommunication : public QObject
{
    Q_OBJECT
private slots:

    void initTestCase();
    void cleanupTestCase();

    void init();
    void cleanup();
    void testReadLocalComputeRemoteWriteLocal();
    void testReadRemoteComputeLocalWriteRemote();
    void testReadRemoteComputeRemoteWriteLocal();
private:
    std::shared_ptr<mapit::Repository> initEmptyLocal();
    std::shared_ptr<mapit::Repository> initNetwork(bool computeLocal);

    //// functions to execute on the repositories
    void readPcd(mapit::Workspace *workspace);
    void voxelgrid(mapit::Workspace* workspace);
    void writePcdLocalOnly(mapit::Workspace* workspace, std::string filename);
    void compareFileToExpected(const char* filename);



    constexpr static float LEAF_SIZE = 0.01;
    constexpr static const char* FILENAME = "data/bunny.pcd";
    constexpr static const char* FILENAME_OUT1 = "bunny_voxelgrid_locally_read_remote_computed.pcd";
    constexpr static const char* FILENAME_OUT2 = "bunny_voxelgrid_remote_read_locally_computed.pcd";
    constexpr static const char* FILENAME_OUT3 = "bunny_voxelgrid_remote_read_remote_computed.pcd";
};

#endif
