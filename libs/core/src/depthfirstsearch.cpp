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

#include <upns/depthfirstsearch.h>
#include <upns/errorcodes.h>

namespace upns
{

using namespace mapit::msgs;

StatusCode depthFirstSearch(CheckoutCommon *checkout, std::shared_ptr<Entity> obj, const ObjectReference &ref, const Path& path,
                            std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> afterCommit,
                            std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree,
                            std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    assert(obj != NULL);
    if(!beforeEntity(obj, ref, path))
    {
        afterEntity(obj, ref, path);
        return UPNS_STATUS_OK;
    }
    //TODO: Entitydata!
    if(!afterEntity(obj, ref, path)) return UPNS_STATUS_OK;
    return UPNS_STATUS_OK;
}

StatusCode depthFirstSearch(CheckoutCommon *checkout, std::shared_ptr<Tree> obj, const ObjectReference &ref, const Path& path,
                                                std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> afterCommit,
                                                std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree,
                                                std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    if(obj == nullptr) return UPNS_STATUS_ERROR;
    //assert(obj != NULL);
    if(!beforeTree(obj, ref, path))
    {
        afterTree(obj, ref, path);
        return UPNS_STATUS_OK;
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
            return UPNS_STATUS_ERR_DB_CORRUPTION;
//            std::shared_ptr<Commit> commit(checkout->getCommit(childoid));
//            StatusCode s = depthFirstSearch(checkout, commit, childoid, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
//            if(!upnsIsOk(s)) return s;
        }
        else if(t == MessageType::MessageTree)
        {
            std::shared_ptr<Tree> tree(checkout->getTree(childpath));
            StatusCode s = depthFirstSearch(checkout, tree, childref, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
            if(!upnsIsOk(s)) return s;
        }
        else if(t == MessageType::MessageEntity)
        {
            std::shared_ptr<Entity> entity(checkout->getEntity(childpath));
            StatusCode s = depthFirstSearch(checkout, entity, childref, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
            if(!upnsIsOk(s)) return s;
        }
        else
        {
            log_warn("Unsupported type during depth search: " + iter->first + ".\nYou probably changed (deleted) filed in the mapit folder manually.");
        }
        iter++;
    }
    if(!afterTree(obj, ref, path)) return UPNS_STATUS_OK; //TODO: return error code.
    return UPNS_STATUS_OK;
}

StatusCode depthFirstSearch(CheckoutCommon *checkout, std::shared_ptr<Commit> obj, const ObjectReference &ref, const Path& path,
                            std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> afterCommit,
                            std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree,
                            std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    assert(obj != NULL);
    if(!beforeCommit(obj, ref, path))
    {
        afterCommit(obj, ref, path);
        return UPNS_STATUS_OK;
    }

    std::shared_ptr<Tree> tree(checkout->getRoot());
    if( tree )
    {
        StatusCode s = depthFirstSearch(checkout, tree, obj->root(), "/", beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
        if(!upnsIsOk(s)) return s;
    }
    if(!afterCommit(obj, ref, path)) return UPNS_STATUS_OK;
    return UPNS_STATUS_OK;
}

StatusCode depthFirstSearch(  upns::CheckoutCommon *checkout
                            , std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> beforeCommit
                            , std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> afterCommit
                            , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree
                            , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree
                            , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity
                            , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    ObjectReference nullRef;
    return depthFirstSearch(checkout, checkout->getRoot(), nullRef, "", beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
}

StatusCode depthFirstSearch(  upns::CheckoutCommon *checkout
                            , const Path& path
                            , std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> beforeCommit
                            , std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> afterCommit
                            , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree
                            , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree
                            , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity
                            , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    if (path.empty()) {
        return depthFirstSearch(checkout, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
    } else {
        std::shared_ptr<Tree> tree = checkout->getTree(path);
        if ( ! tree ) {
            return UPNS_STATUS_ENTITY_NOT_FOUND; // TODO: actuly tree not found
        }
        ObjectReference nullRef;
        return depthFirstSearch(checkout, tree, nullRef, path, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
    }
}

}
