/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "repositoryimpl.h"
#include "serialization/abstractserializer.h"
#include "versioning/workspaceimpl.h"
#include <mapit/depthfirstsearch.h>
#include <mapit/serialization/entitydatalibrarymanager.h>
#include <chrono>

namespace mapit
{
class RepositoryPrivate
{
    RepositoryPrivate(std::shared_ptr<AbstractSerializer> ser):m_serializer(ser){}

    std::shared_ptr<AbstractSerializer> m_serializer;
    friend class RepositoryImpl;
};

RepositoryImpl::RepositoryImpl(std::shared_ptr<AbstractSerializer> serializer)
    :m_p(new RepositoryPrivate(serializer))
{}

RepositoryImpl::~RepositoryImpl()
{
    delete m_p;
}

std::shared_ptr<Workspace> RepositoryImpl::createWorkspace(const CommitId &commitIdOrBranchname, const std::string &name)
{
    std::shared_ptr<WorkspaceObj> workspace(m_p->m_serializer->getworkspaceCommit(name));
    if(workspace != NULL)
    {
        log_warn("Workspace with this name already exist: " + name);
        return NULL;
    }
    std::shared_ptr<Branch> branch(m_p->m_serializer->getBranch(commitIdOrBranchname));
    CommitId commitId;
    std::string branchName;
    if(branch != NULL)
    {
        // assert: empty, if this is the inial commit and "master"
        assert( branch->commitid().empty() || m_p->m_serializer->getCommit(branch->commitid()) != NULL );
        if(branch->commitid().empty())
        {
            log_info("empty repository. checking out initial master");
        }
        commitId = branch->commitid();
        branchName = commitIdOrBranchname;
    }
    else
    {
        std::shared_ptr<Commit> commit(m_p->m_serializer->getCommit(commitIdOrBranchname));
        if(commit != NULL)
        {
            commitId = commitIdOrBranchname;
            branchName = "";
        }
        else
        {
            log_error("given commitIdOrBranchname was not a commitId or branchname.");
            return NULL;
        }
    }
    workspace = std::shared_ptr<WorkspaceObj>(new WorkspaceObj());
    workspace->mutable_rollingcommit()->add_parentcommitids(commitId);
    StatusCode s = m_p->m_serializer->createworkspaceCommit( workspace, name );
    if(!mapitIsOk(s))
    {
        log_error("Could not create workspace.");
    }
    return std::shared_ptr<Workspace>(new WorkspaceImpl(m_p->m_serializer, workspace, name, branchName));
}

std::vector<std::string> RepositoryImpl::listWorkspaceNames()
{
    return m_p->m_serializer->listWorkspaceNames();
}

std::shared_ptr<Tree> RepositoryImpl::getTree(const ObjectId &oid)
{
    return m_p->m_serializer->getTree(oid);
}

std::shared_ptr<Entity> RepositoryImpl::getEntity(const ObjectId &oid)
{
    return m_p->m_serializer->getEntity(oid);
}

std::shared_ptr<Commit> RepositoryImpl::getCommit(const CommitId &oid)
{
    return m_p->m_serializer->getCommit(oid);
}

std::shared_ptr<WorkspaceObj> RepositoryImpl::getWorkspaceObj(const std::string &name)
{
    return m_p->m_serializer->getworkspaceCommit(name);
}

std::shared_ptr<Branch> RepositoryImpl::getBranch(const std::string &name)
{
    return m_p->m_serializer->getBranch(name);
}

MessageType RepositoryImpl::typeOfObject(const ObjectId &oid)
{
    return m_p->m_serializer->typeOfObject(oid);
}

std::shared_ptr<AbstractEntitydata> RepositoryImpl::getEntitydataReadOnly(const ObjectId &oid)
{
    // For entitydata it is not enough to call serializer directly.
    // Moreover special classes need to be created by layertype plugins.
    std::shared_ptr<Entity> ent = m_p->m_serializer->getEntity( oid );
    if( ent == NULL )
    {
        log_error("Entity not found." + oid);
        return NULL;
    }
    assert( ent );
    return EntityDataLibraryManager::getEntitydataFromProvider(ent->type(), m_p->m_serializer->getStreamProvider(ent->dataid(), true), true);
}

std::shared_ptr<Workspace> RepositoryImpl::getWorkspace(const std::string &workspaceName)
{
    std::shared_ptr<WorkspaceObj> workspace(m_p->m_serializer->getworkspaceCommit(workspaceName));
    if(workspace == NULL)
    {
        log_info("Workspace does not exist: " + workspaceName);
        return NULL;
    }
    return std::shared_ptr<Workspace>(new WorkspaceImpl(m_p->m_serializer, workspace, workspaceName));
}

StatusCode RepositoryImpl::deleteWorkspaceForced(const std::string &workspaceName)
{
    std::shared_ptr<WorkspaceObj> workspace(m_p->m_serializer->getworkspaceCommit(workspaceName));
    if(workspace == NULL)
    {
        log_info("Workspace with this name does not exist: " + workspaceName);
        return MAPIT_STATUS_ENTITY_NOT_FOUND;
    }
    //TODO: Get workspace, remove its inner Commit and objects (objects only if not referenced)!
    return m_p->m_serializer->removeWorkspace(workspaceName);
}

CommitId RepositoryImpl::commit(const std::shared_ptr<Workspace> workspace, std::string msg, std::string author, std::string email, mapit::time::Stamp stamp)
{
    size_t logStatsFileChanged = 0;

    WorkspaceImpl *ws = static_cast<WorkspaceImpl*>(workspace.get());
    std::map< Path, ObjectId > oldPathsToNewOids;
    CommitId refID;
    std::shared_ptr<Commit> rollingCommit(new Commit(ws->getWorkspaceObj()->rollingcommit()));
    ObjectReference nullRef;
    StatusCode s = mapit::depthFirstSearchWorkspace(
                ws,
                ws->getRoot(),
                nullRef,
                "/",
        depthFirstSearchWorkspaceAll(Tree),
        [&](std::shared_ptr<Tree> tree, const ObjectReference &ref, const Path &path)
        {
            assert(tree != NULL);
            ::google::protobuf::Map< ::std::string, ::mapit::msgs::ObjectReference > &refs = *tree->mutable_refs();
            ::google::protobuf::Map< ::std::string, ::mapit::msgs::ObjectReference >::iterator iter(refs.begin());
            while(iter != refs.end())
            {
                if ( iter->second.path().empty()) {
                    assert(! iter->second.id().empty());
                } else {
                    Path childPath(iter->second.path());
                    childPath = childPath.substr(childPath.find_first_of("/"), childPath.length());
                    if (childPath.back() == '/') {
                        childPath = childPath.substr(0, childPath.find_last_not_of("/")+1);
                    }
                    assert(oldPathsToNewOids.find(childPath) != oldPathsToNewOids.end());
                    const ObjectId &newId = oldPathsToNewOids[childPath];
                    assert(!newId.empty());
                    if(newId != iter->second.id())
                    {
                        iter->second.set_id(newId);
                        iter->second.clear_path();
                    }
                }

                iter++;
            }
            std::pair<StatusCode, ObjectId> statusOid = m_p->m_serializer->storeTree(tree);
            if ( ! mapitIsOk(statusOid.first)) {
                return false;
            }
            m_p->m_serializer->removeTreeTransient( ref.path() );
            oldPathsToNewOids.insert(std::pair<std::string, std::string>(path, statusOid.second));
            return true;
        },
        depthFirstSearchWorkspaceAll(Entity),
        [&](std::shared_ptr<Entity> entity, const ObjectReference &ref, const Path &path)
        {
            // we have nothing todo when ref.path() is empty
            bool entityExistsTransient     = (! ref.path().empty()) && (m_p->m_serializer->getEntityTransient( ref.path() ) != nullptr);
            bool entitydataExistsTransient = (! ref.path().empty()) && m_p->m_serializer->existsStreamProviderTransient( ref.path() );

            // first save the data (becase the ID to the new data needs to be in the entity)
            if (entitydataExistsTransient) {
                std::pair<StatusCode, ObjectId> edStatus = m_p->m_serializer->persistTransientEntitydata( ref.path() );
                if ( ! mapitIsOk(edStatus.first) ) {
                    log_error("Commit: error while saving \"" << path << "\" persistent. Code: " << edStatus.first);
                    return false;
                }
                // check if the data ID changed for the entity
                if ( ! edStatus.second.empty() && entity->dataid() != edStatus.second) {
                    entity->set_dataid( edStatus.second );
                    entityExistsTransient = true; // might has been true anyways, but now we need to update at least the new reference
                }
            }

            // afterwards save the entity
            if (entityExistsTransient) {
                std::pair<StatusCode, ObjectId> statusOid = m_p->m_serializer->storeEntity(entity);
                if ( ! mapitIsOk(statusOid.first) ) {
                    return false;
                }
                m_p->m_serializer->removeEntityTransient( ref.path() );
                // save ID of entity for tree storage
                oldPathsToNewOids.insert(std::pair<std::string, std::string>(path, statusOid.second));

                logStatsFileChanged++;
            } else {
                // save ID of entity for tree storage
                oldPathsToNewOids.insert(std::pair<std::string, std::string>(path, ref.id()));
            }

            return true;
        });
    if(!mapitIsOk(s))
    {
        log_error("error while commiting the workspace");
    }
    {
        std::shared_ptr<Commit> commit = rollingCommit;
        const Path pathOfRootDir("/");
        bool changes = oldPathsToNewOids.find(pathOfRootDir) != oldPathsToNewOids.end();

        if ( changes ) {
            // add reference to root
            commit->mutable_root()->set_id(oldPathsToNewOids[pathOfRootDir]);
            commit->mutable_root()->clear_path();
        } else {
            // no changes where found
            log_warn("commit empty workspace on empty parent commit (no root)");
        }

        if(msg.find_last_of('\n') != msg.length()-1) {
            msg += "\n";
        }
        commit->set_commitmessage(msg);
        mapit::msgs::Time stampMsg = mapit::time::to_msg( stamp );
        commit->mutable_stamp()->set_sec( stampMsg.sec() );
        commit->mutable_stamp()->set_nsec( stampMsg.nsec() );
        commit->set_author(author + " <" + email + ">");

        std::pair<StatusCode, ObjectId> statusOid = m_p->m_serializer->createCommit(commit);
        if ( ! mapitIsOk(statusOid.first) ) {
            log_error("error while commiting");
        }
        refID = statusOid.second;
    }

    // TODO: assert check if workspace is empty
    // delete all folder of workspace
    std::string coName = ws->getName();
    std::string branchName = ws->getBranchName();

    // update workspace
    ws->getWorkspaceObj()->clear_transientoidstoorigin();
    ws->getWorkspaceObj()->mutable_rollingcommit()->Clear();
    ws->getWorkspaceObj()->mutable_rollingcommit()->add_parentcommitids( refID );
    std::shared_ptr<Commit> commit = getCommit(refID);
    assert(commit);
    ws->getWorkspaceObj()->mutable_rollingcommit()->mutable_root()->set_id( commit->root().id() );
    assert(commit->root().path().empty());
    StatusCode status = m_p->m_serializer->storeWorkspaceCommit( ws->getWorkspaceObj(), coName );
    if ( ! mapitIsOk(status) ) {
        log_error("Could not update workspace.");
    }

    log_info("");
    log_info("[" << branchName << " " << refID.substr(0, 7) << "] " << msg.substr(0, msg.find_first_of("\n") ));
    log_info("" << logStatsFileChanged << " entity added"); // TODO to check if something was removed, one would had to compare the tree childen from the transient with persistent
    log_info("");
    log_info("\tCommited " << coName << " -> " << branchName);
    log_info("");

    return refID;
}

std::vector<std::shared_ptr<Branch> > RepositoryImpl::getBranches()
{
    return std::vector<std::shared_ptr<Branch> >();
}

StatusCode RepositoryImpl::push(Repository &repo)
{
    return MAPIT_STATUS_OK;
}

StatusCode RepositoryImpl::pull(Repository &repo)
{
    return MAPIT_STATUS_OK;
}

CommitId RepositoryImpl::parseCommitRef(const std::string &commitRef)
{
    return "";
}

std::shared_ptr<Workspace> RepositoryImpl::merge(const CommitId mine, const CommitId theirs, const CommitId base)
{
    return NULL;
}

std::vector<std::pair<CommitId, ObjectId> > RepositoryImpl::ancestors(const CommitId &commitId, const ObjectId &objectId, const int level)
{
    return std::vector<std::pair<CommitId, ObjectId> >();
}

bool RepositoryImpl::canRead()
{
    return true;
}

bool RepositoryImpl::canWrite()
{
    return true;
}

}
