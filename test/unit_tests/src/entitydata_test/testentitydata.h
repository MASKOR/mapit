/*******************************************************************************
 *
 * Copyright 2015-2016 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2015 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef __TESTENTITYDATA_H
#define __TESTENTITYDATA_H

#include <QTest>

#include "versioning/repository.h"
#include "../repositorycommon.h"

namespace mapit {
class AbstractEntityData;
class AbstractMapSerializer;
class AbstractEntityDataStreamProvider;
}

class TestEntitydata : public QObject
{
    Q_OBJECT
private slots:
    void init();
    void cleanup();

    void initTestCase();
    void cleanupTestCase();

    void testCreateLayer_data();
    void testCreateLayer();
private:
    void createTestdata();
    mapit::upnsSharedPointer<mapit::AbstractMapSerializer> m_ed[2];
    mapit::upnsSharedPointer<mapit::AbstractEntityDataStreamProvider> m_edsp[2];

};

#endif
