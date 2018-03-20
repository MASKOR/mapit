/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2016-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef FS_SERIALIZER_H
#define FS_SERIALIZER_H

#include <mapit/typedefs.h>
#include <mapit/logging.h>
#include "serialization/abstractserializer.h"
#include <mapit/operators/serialization/abstractentitydataprovider.h>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

class QLockFile;

namespace fs = boost::filesystem;

using namespace mapit::msgs;

namespace mapit
{


  /**
 * @brief The FSSerializer class stores all data using a file system.
 *
 */

  class FSSerializer : public AbstractSerializer
  {
  private:
    static const fs::path _PREFIX_MAPIT_;
    static const fs::path _PREFIX_TREE_;
    static const fs::path _PREFIX_ENTITY_;
    static const fs::path _PREFIX_ENTITY_DATA_;
    static const fs::path _PREFIX_WORKSPACES_;
    static const fs::path _PREFIX_BRANCHES_;
    static const fs::path _PREFIX_COMMIT_;

//    static const fs::path _FILE_workspace_;
    static const fs::path _PREFIX_workspace_;
    static const fs::path _WORKSPACE_GENERIC_ENTRY_;
    static const fs::path _workspace_ENTITY_DATA_;
    static const fs::path _workspace_ROOT_FOLDER_;
  private:
    fs::path repo_;
  private:
    fs::path objectid_to_workspace_fs_path(ObjectId oid);
    fs::path path_to_commit_fs_path(const Path& path, const fs::path& prefix);
    void fs_check_create(fs::path path);
    void fs_write(fs::path path, std::shared_ptr<GenericEntry> ge, MessageType msgType, bool overwrite = false);
    void fs_read(fs::path path, std::shared_ptr<GenericEntry> entry);
    void fs_delete(fs::path path);
  public:

    FSSerializer(std::string directory);
    virtual ~FSSerializer();

    virtual bool canRead();
    virtual bool canWrite();

    virtual std::shared_ptr<Tree> getTree(const ObjectId &oid);
    virtual std::shared_ptr<Tree> getTreeTransient(const PathInternal &transientId);
    virtual std::pair<StatusCode, ObjectId> storeTree(std::shared_ptr<Tree> obj);
    virtual std::pair<StatusCode, ObjectId> storeTreeTransient(std::shared_ptr<Tree> obj, const PathInternal &transientId);
    //virtual std::pair<StatusCode, ObjectId> createTreeTransient(std::shared_ptr<Tree> &obj, const Path &path);
    virtual StatusCode removeTreeTransient(const PathInternal &transientId);

    virtual std::shared_ptr<mapit::msgs::Entity> getEntity(const ObjectId oid);
    virtual std::shared_ptr<mapit::msgs::Entity> getEntityTransient(const PathInternal oid);
    virtual std::pair<StatusCode, ObjectId> storeEntity(std::shared_ptr<mapit::msgs::Entity> obj);
    virtual std::pair<StatusCode, ObjectId> storeEntityTransient(std::shared_ptr<mapit::msgs::Entity> obj, const PathInternal &transientId);
    //virtual StatusCode createEntityTransient(std::shared_ptr<Entity> &obj);
    virtual StatusCode removeEntityTransient(const PathInternal &transientId);

    virtual std::shared_ptr<Commit> getCommit(const ObjectId &oid);
    //virtual std::pair<StatusCode, ObjectId> storeCommit(std::shared_ptr<Commit> &obj);
    virtual std::pair<StatusCode, ObjectId> createCommit(std::shared_ptr<Commit> obj);
    virtual StatusCode removeCommit(const ObjectId &oid);

    virtual std::vector< std::string > listWorkspaceNames();
    virtual std::vector< std::shared_ptr<WorkspaceObj> > listWorkspaces();
    virtual std::shared_ptr<WorkspaceObj> getworkspaceCommit(const std::string &name);
    virtual StatusCode storeWorkspaceCommit(std::shared_ptr<WorkspaceObj> obj, const std::string &name);
    virtual StatusCode createworkspaceCommit(std::shared_ptr<WorkspaceObj> obj, const std::string &name);
    virtual StatusCode removeWorkspace(const std::string &name);

    virtual std::vector< std::shared_ptr<Branch> > listBranches();
    virtual std::shared_ptr<Branch> getBranch(const std::string &name);
    virtual StatusCode storeBranch(std::shared_ptr<Branch> obj, const std::string &name);
    virtual StatusCode createBranch(std::shared_ptr<Branch> obj, const std::string &name);
    virtual StatusCode removeBranch(const std::string &name);

    virtual bool existsStreamProvider(const ObjectId &entityId);
    virtual bool existsStreamProviderTransient(const Path &path);
    virtual std::shared_ptr<AbstractEntitydataProvider> getStreamProvider(const ObjectId &entityId, bool canRead);
    virtual std::shared_ptr<AbstractEntitydataProvider> getStreamProviderTransient(const Path &oid, bool canRead, bool canWrite);

    /**
     * @brief cleanUp Collects Grabage. Orphan Objects, not reachable by any branch are removed.
     * @return
     */
    virtual StatusCode cleanUp();

    virtual MessageType typeOfObject(const ObjectId &oid);
    virtual MessageType typeOfObjectTransient(const PathInternal &pathIntenal);
    virtual bool exists(const ObjectId &oidOrName);

    virtual std::pair<StatusCode, ObjectId> persistTransientEntitydata(const PathInternal &pathInternal);
#ifdef MAPIT_DEBUG
    virtual void debugDump();
#endif
  };

}
#endif
