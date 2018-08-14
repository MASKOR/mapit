/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef __GRID2DTEST_H
#define __GRID2DTEST_H

#include <QTest>
#include <mapit/layertypes/grid2d.h>
#include "../repositorycommon.h"

class TestGrid2D: public RepositoryCommon
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void testGetData();
    void testSetData();

    void testGetData_data();
    void testSetData_data();
};

#endif
