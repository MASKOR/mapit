/*******************************************************************************
 *
 * Copyright      2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef BREADTHFIRSTSEARCH_H
#define BREADTHFIRSTSEARCH_H

#include <mapit/typedefs.h>
#include <mapit/logging.h>
#include <mapit/msgs/services.pb.h>
#include <mapit/versioning/workspace.h>
#include <mapit/versioning/repository.h>

namespace mapit
{

StatusCode breadthFirstSearchHistory(  std::shared_ptr<Repository> repo
                                     , const CommitId& commitID
                                     , msgs::Commit commit
                                     , std::function<bool(std::shared_ptr<mapit::msgs::Commit>, const mapit::CommitId&)> beforeCommit
                                     , std::function<bool(std::shared_ptr<mapit::msgs::Commit>, const mapit::CommitId&)> afterCommit
                                    );

#define breadthFirstSearcCommitContinue [](std::shared_ptr<mapit::msgs::Commit> commit,  const mapit::CommitId& commitID){return true;}
#define breadthFirstSearcCommitStop [](std::shared_ptr<mapit::msgs::Commit> commit,  const mapit::CommitId& commitID){return false;}

}

#endif
