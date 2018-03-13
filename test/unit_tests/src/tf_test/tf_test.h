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

#ifndef __TF_TEST_H
#define __TF_TEST_H

#include <QTest>
#include "../repositorycommon.h"

namespace upns {
namespace tf {
    struct TransformStamped;
}
}

class TFTest : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void test_input_output();
    void test_chain_of_2_tfs();
    void test_interpolation();

    void test_layertype_to_buffer();
private:
    std::string fileSystemName_;
    std::shared_ptr<upns::Repository> repo_;
    std::shared_ptr<upns::Checkout> checkout_;

    void createTestdata();

    void compareTfs(upns::tf::TransformStamped a, upns::tf::TransformStamped b);
};

#endif
