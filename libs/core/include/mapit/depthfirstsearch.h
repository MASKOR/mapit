/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef DEPTHFIRSTSEARCH_H
#define DEPTHFIRSTSEARCH_H

#include <mapit/typedefs.h>
#include <mapit/logging.h>
#include <mapit/msgs/services.pb.h>
#include <mapit/versioning/workspace.h>
#include <mapit/versioning/repository.h>

namespace mapit
{
/**
 * @brief Depth first search for Commit, Tree and Entity. This is often very handy.
 * Does not work for branches. Does not visit Entitydata (must be done manually).
 */
StatusCode depthFirstSearchWorkspace(  mapit::WorkspaceCommon *workspace
                                     , std::shared_ptr<mapit::msgs::Tree> obj
                                     , const ObjectReference &ref
                                     , const Path& path
                                     , std::function<bool(std::shared_ptr<mapit::msgs::Tree>, const ObjectReference&, const Path&)> beforeTree
                                     , std::function<bool(std::shared_ptr<mapit::msgs::Tree>, const ObjectReference&, const Path&)> afterTree
                                     , std::function<bool(std::shared_ptr<mapit::msgs::Entity>, const ObjectReference&, const Path&)> beforeEntity
                                     , std::function<bool(std::shared_ptr<mapit::msgs::Entity>, const ObjectReference&, const Path&)> afterEntity);

StatusCode depthFirstSearchWorkspace(  mapit::WorkspaceCommon *workspace
                                     , std::shared_ptr<mapit::msgs::Entity> obj
                                     , const ObjectReference &ref, const Path& path
                                     , std::function<bool(std::shared_ptr<mapit::msgs::Tree>, const ObjectReference&, const Path&)> beforeTree
                                     , std::function<bool(std::shared_ptr<mapit::msgs::Tree>, const ObjectReference&, const Path&)> afterTree
                                     , std::function<bool(std::shared_ptr<mapit::msgs::Entity>, const ObjectReference&, const Path&)> beforeEntity
                                     , std::function<bool(std::shared_ptr<mapit::msgs::Entity>, const ObjectReference&, const Path&)> afterEntity);

StatusCode depthFirstSearchWorkspace(  mapit::WorkspaceCommon *workspace
                                     , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree
                                     , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree
                                     , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity
                                     , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity);

StatusCode depthFirstSearchWorkspace(  mapit::WorkspaceCommon *workspace
                                     , const Path& path
                                     , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree
                                     , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree
                                     , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity
                                     , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity);

// TODO change "mapit::WorkspaceCommon*" to "const mapit::WorkspaceCommon&"

StatusCode depthFirstSearchHistory(  std::shared_ptr<Repository> repo
                                   , const CommitId& commitID
                                   , std::function<bool(std::shared_ptr<mapit::msgs::Commit>, const mapit::CommitId&)> beforeCommit
                                   , std::function<bool(std::shared_ptr<mapit::msgs::Commit>, const mapit::CommitId&)> afterCommit
                                  );

#define depthFirstSearchWorkspaceAll(c) ([](std::shared_ptr<c> obj, const ObjectReference& ref, const mapit::Path &path){return true;})
#define depthFirstSearchHistoryAll [](std::shared_ptr<mapit::msgs::Commit> commit,  const mapit::CommitId& commitID){return true;}

}

#endif
