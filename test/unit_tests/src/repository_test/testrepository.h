/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2016 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef TESTREPOSITORY_H
#define TESTREPOSITORY_H

#include <QTest>

#include <upns/versioning/repository.h>
#include "../repositorycommon.h"

namespace upns {
class RepositoryServer;
}

class TestRepository : public RepositoryCommon
{
private:
    std::string filename_;
    std::string checkoutPath_;
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void testCreateCheckout_data();
    void testCreateCheckout();
    void testGetCheckout_data();
    void testGetCheckout();
    void testReadCheckout_data();
    void testReadCheckout();
    void testCommit_data();
    void testCommit();
    void testVoxelgridfilter_data();
    void testVoxelgridfilter();
};

#endif
