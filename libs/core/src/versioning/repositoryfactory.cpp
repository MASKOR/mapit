/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2016-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "upns/versioning/repositoryfactory.h"
#include "serialization/abstractserializer.h"
#include "serialization/file_system/fs_serializer.h"
#include "repositoryimpl.h"

static upns::AbstractSerializer *initializeSerializer(std::string directory)
{
    upns::AbstractSerializer *mser = new upns::FSSerializer(directory);

    // Check if anything exists in the database
    // Note: There might be commits or objects which are not recognized here.
    // TODO: forbid to delete last branch for this to work. Checkouts might all be deleted.
    size_t numElems = mser->listBranches().size();

    if(numElems) return mser;
//        numElems = m_serializer->listCheckoutNames().size();
//        if(numElems) return;
    log_warn("Selected empty repository, create master");
    std::shared_ptr<mapit::msgs::Branch> master(new mapit::msgs::Branch());
    master->set_commitid(""); //< InitialCommit
    mser->createBranch(master, "master");
    return mser;
}

upns::Repository *upns::RepositoryFactory::openLocalRepository(std::string directory)
{
    std::shared_ptr<AbstractSerializer> mser( initializeSerializer( directory ) );
    return new upns::RepositoryImpl( mser );
}
