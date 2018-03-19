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

#include <mapit/depthfirstsearch.h>
#include <mapit/errorcodes.h>

namespace mapit
{

using namespace mapit::msgs;

StatusCode depthFirstSearchWorkspace(  CheckoutCommon *checkout
                                     , std::shared_ptr<Entity> obj
                                     , const ObjectReference &ref
                                     , const Path& path
                                     , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree
                                     , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree
                                     , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity
                                     , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    assert(obj != NULL);
    if(!beforeEntity(obj, ref, path))
    {
        afterEntity(obj, ref, path);
        return MAPIT_STATUS_OK;
    }
    //TODO: Entitydata!
    if(!afterEntity(obj, ref, path)) return MAPIT_STATUS_OK;
    return MAPIT_STATUS_OK;
}

StatusCode depthFirstSearchWorkspace(CheckoutCommon *checkout
                                     , std::shared_ptr<Tree> obj
                                     , const ObjectReference &ref
                                     , const Path& path
                                     , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree
                                     , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree
                                     , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity
                                     , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    if(obj == nullptr) return MAPIT_STATUS_ERROR;
    //assert(obj != NULL);
    if(!beforeTree(obj, ref, path))
    {
        afterTree(obj, ref, path);
        return MAPIT_STATUS_OK;
    }
    ::google::protobuf::Map< ::std::string, ::mapit::msgs::ObjectReference > &refs = *obj->mutable_refs();
    std::map<std::string, ::mapit::msgs::ObjectReference> refsSorted;
    for (std::pair<::std::string, ::mapit::msgs::ObjectReference> refsElement : refs) {
        refsSorted.insert(refsElement);
    }
    std::map< ::std::string, ::mapit::msgs::ObjectReference >::iterator iter(refsSorted.begin());
    while(iter != refsSorted.cend())
    {
        const ObjectReference &childref = iter->second;
        const Path &childpath = (path=="/"?"":path) + "/" + iter->first;
        MessageType t = checkout->typeOfObject(childpath);

        if(t == MessageType::MessageCommit)
        {
            assert(false);
            log_error("Commit found in tree. Commit must be root");
            return MAPIT_STATUS_ERR_DB_CORRUPTION;
        }
        else if(t == MessageType::MessageTree)
        {
            std::shared_ptr<Tree> tree(checkout->getTree(childpath));
            StatusCode s = depthFirstSearchWorkspace(checkout, tree, childref, childpath, beforeTree, afterTree, beforeEntity, afterEntity);
            if(!mapitIsOk(s)) return s;
        }
        else if(t == MessageType::MessageEntity)
        {
            std::shared_ptr<Entity> entity(checkout->getEntity(childpath));
            StatusCode s = depthFirstSearchWorkspace(checkout, entity, childref, childpath, beforeTree, afterTree, beforeEntity, afterEntity);
            if(!mapitIsOk(s)) return s;
        }
        else
        {
            log_warn("Unsupported type during depth search: " + iter->first + ".\nYou probably changed (deleted) filed in the mapit folder manually.");
        }
        iter++;
    }
    if(!afterTree(obj, ref, path)) return MAPIT_STATUS_OK; //TODO: return error code.
    return MAPIT_STATUS_OK;
}

StatusCode depthFirstSearchWorkspace(  mapit::CheckoutCommon *checkout
                                     , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree
                                     , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree
                                     , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity
                                     , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    ObjectReference nullRef;
    return depthFirstSearchWorkspace(checkout, checkout->getRoot(), nullRef, "", beforeTree, afterTree, beforeEntity, afterEntity);
}

StatusCode depthFirstSearchWorkspace(  mapit::CheckoutCommon *checkout
                                     , const Path& path
                                     , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree
                                     , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree
                                     , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity
                                     , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    if (path.empty()) {
        return depthFirstSearchWorkspace(checkout, beforeTree, afterTree, beforeEntity, afterEntity);
    } else {
        std::shared_ptr<Tree> tree = checkout->getTree(path);
        if ( ! tree ) {
            return MAPIT_STATUS_ENTITY_NOT_FOUND; // TODO: actuly tree not found
        }
        ObjectReference nullRef;
        return depthFirstSearchWorkspace(checkout, tree, nullRef, path, beforeTree, afterTree, beforeEntity, afterEntity);
    }
}

StatusCode depthFirstSearchHistory(  std::shared_ptr<mapit::Repository> repo
                                   , const CommitId& commitID
                                   , std::function<bool(std::shared_ptr<Commit>, const CommitId&)> beforeCommit
                                   , std::function<bool(std::shared_ptr<Commit>, const CommitId&)> afterCommit
                                  )
{
    std::shared_ptr<Commit> commit = repo->getCommit(commitID);
    assert(commit);

    bool continueSearchBefore = beforeCommit(commit, commitID);
    if (continueSearchBefore) {
        for (auto parentCommitID : commit->parentcommitids()) {
            if ( ! parentCommitID.empty() ) {
                depthFirstSearchHistory(repo, parentCommitID, beforeCommit, afterCommit);
            }
        }
        afterCommit(commit, commitID);
    }
}

}
