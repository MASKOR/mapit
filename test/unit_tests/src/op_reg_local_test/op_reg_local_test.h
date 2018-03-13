/*******************************************************************************
 *
 * Copyright      2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef __OP_REG_LOCAL_ICP_TEST_H
#define __OP_REG_LOCAL_ICP_TEST_H

#include <QTest>
#include "../repositorycommon.h"

class OPRegLocalICPTest : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void test_icp_tf_add();
    void test_icp_for_more_than_one_input();
    void test_icp_tf_combine();
private:
    std::string fileSystemName_;
    std::shared_ptr<upns::Repository> repo_;
    std::shared_ptr<upns::Checkout> checkout_;

    void createTestdata();
};

#endif
