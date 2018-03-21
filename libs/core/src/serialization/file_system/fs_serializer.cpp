/*******************************************************************************
 *
 * Copyright 2016-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
 *           2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#include "fs_serializer.h"

#include "util.h"
#include "../hash.h"
#include "fs_entitydatastreamprovider.h"
#include <assert.h>
#include <mapit/msgs/services.pb.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <mapit/errorcodes.h>

namespace mapit
{
const fs::path FSSerializer::_PREFIX_MAPIT_       = "/.mapit";
const fs::path FSSerializer::_PREFIX_TREE_        = "/trees/";
const fs::path FSSerializer::_PREFIX_ENTITY_      = "/entities/";
const fs::path FSSerializer::_PREFIX_ENTITY_DATA_ = "/entities_data/";
const fs::path FSSerializer::_PREFIX_BRANCHES_    = "/branches/";
const fs::path FSSerializer::_PREFIX_COMMIT_      = "/commits/";

const fs::path FSSerializer::_PREFIX_WORKSPACES_       = "workspaces";
const fs::path FSSerializer::_WORKSPACE_GENERIC_ENTRY_ = ".generic_entry";
const fs::path FSSerializer::_workspace_ENTITY_DATA_   = "entity_data";
const fs::path FSSerializer::_workspace_ROOT_FOLDER_   = "root";

FSSerializer::FSSerializer(std::string directory)
{
    fs::path path( directory );
    //log_info("Use repo at " + path.string());

    // search for repo
    path = path / _PREFIX_MAPIT_;
    if( ! fs::exists( fs::system_complete( path ) ) ) {
        if ( ! fs::create_directories( path ) ) {
            log_error("Could not create " + path.string() );
        }
    }
//        while ( ! fs::exists( fs::system_complete( path.string() + _PREFIX_MAPIT_ ) ) ) {
//            path = path.parent_path();

//            if ( ! fs::exists( path ) ) {
//                log_info("Could not find " + _PREFIX_MAPIT_ + " in this or its parent folders");
//                path = fs::current_path();
//                log_info("Create " + path.string() + _PREFIX_MAPIT_);
//                if ( ! fs::create_directories( path.string() + _PREFIX_MAPIT_)) {
//                    log_error("Could not create " + path.string() + _PREFIX_MAPIT_);
//                }
//                break;
//            }
//        }

    //log_info("Use " + path.string() + " as repo");
    repo_ = path;

    // TODO do I need to lock this repo?
}

FSSerializer::~FSSerializer()
{

}

void
FSSerializer::fs_check_create(fs::path path)
{
    boost::system::error_code ec;
    if ( ! fs::exists( path ) ) {
        if ( ! fs::create_directories( path, ec ) && ec.value() != 0) { //  && ec.value() != 0 (0 means no error) is only needed since this bug the https://svn.boost.org/trac/boost/ticket/7258
            log_error("Can not create: " + path.string() + " code: " + ec.message());
        } else {
            //log_info("Create: " + path.string());
        }
    }
}

fs::path
FSSerializer::path_to_commit_fs_path(const Path& path, const fs::path& prefix)
{
    fs::path pathInternal = repo_ / _PREFIX_COMMIT_ / prefix / path;

    return pathInternal.remove_trailing_separator();
}

fs::path
FSSerializer::objectid_to_workspace_fs_path(ObjectId oid)
{
    fs::path path = repo_ / _PREFIX_WORKSPACES_;

    size_t seperator_pose = oid.find("/");
    if (seperator_pose != std::string::npos) {
        path /= fs::path(oid.substr(0, seperator_pose));                    // first part is the name of the workspace name
        path /= _workspace_ROOT_FOLDER_;
        path /= oid.substr(seperator_pose, oid.size() - seperator_pose);    // second part is the id of the file within the workspace which is used as a path in the filesystem
    } else {
        path /= fs::path(oid);
//        log_warn("Can't find \"/\" in ObjectId of object in workspace, don't extended path as <workspaceName>/" + _workspace_ROOT_FOLDER_.string() + "/<checkoutid>\n the path is: " + path.string());
    }

    return path.remove_trailing_separator();
}

void
FSSerializer::fs_write(fs::path path, std::shared_ptr<GenericEntry> ge, MessageType msgType, bool overwrite )
{
    if ( fs::exists( path ) ) {
        if ( ! overwrite) {
            log_warn("Duplicate found " + path.string());
            return;
        } else {
            //log_info("Overwrite duplicate " + path.string());
        }
    }

    ge->set_type(msgType);

    //log_info("Write type \"" + MessageType_Name(msgType) + "\" to " + path.string());
    create_directories( path.parent_path() );

    std::filebuf buffer;
    buffer.open(path.string(), std::ios::out);

    std::ostream stream(&buffer);
    stream << ge->SerializeAsString();

    buffer.close();
}

void
FSSerializer::fs_read(fs::path path, std::shared_ptr<GenericEntry> entry)
{
    if ( ! fs::exists( path ) ) {
        log_error("Can't read " + path.string());
        return;
    }

    std::ifstream is(path.string());
    std::stringstream strs;
//    std::filebuf buffer;
//    buffer.open(path.string(), std::ios::in);
//    std::string entry_serialized;
//    while (stream) {
//        entry_serialized += char(stream.get());
//    }
    char buffer[1024];
    while(is.read(buffer, 1024)) {
        std::streamsize size=is.gcount();
        strs.write(buffer, size);
    }
    std::streamsize size=is.gcount();
    if(size > 0)
    {
        strs.write(buffer, size);
    }
//    buffer.close();

    entry->ParseFromString( strs.str() );
}

void
FSSerializer::fs_delete(fs::path path)
{
    fs::remove(path);
}

std::shared_ptr<Tree>
FSSerializer::getTree(const ObjectId &oid)
{
    fs::path path( repo_ / _PREFIX_TREE_ / oid);

    if ( ! fs::exists( path ) ) {
        return std::shared_ptr<Tree>(NULL);
    }

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    fs_read(path, ge);

    if ( ge->type() != MessageTree ) {
//        log_debug("Tree at: " + path.string() + " has variable type not set correctly");
        return nullptr;
    }
    if ( ! ge->has_tree() ) {
        log_fatal("Tree at: " + path.string() + " is no tree");
        return std::shared_ptr<Tree>(NULL);
    }

    return std::shared_ptr<Tree>(new Tree( ge->tree() ));
}

std::shared_ptr<Tree>
FSSerializer::getTreeTransient(const PathInternal &transientId)
{
    fs::path path = objectid_to_workspace_fs_path( transientId ) / _WORKSPACE_GENERIC_ENTRY_;

    if ( ! fs::exists( path ) ) {
        return std::shared_ptr<Tree>(NULL);
    }

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    fs_read(path, ge);

    if ( ge->type() != MessageTree ) {
        //log_warn("Tree at: " + path.string() + " has variable type not set correctly");
        return nullptr;
    }
    if ( ! ge->has_tree() ) {
        log_fatal("Tree at: " + path.string() + " is no tree");
        return std::shared_ptr<Tree>(NULL);
    }

    return std::shared_ptr<Tree>(new Tree( ge->tree() ));
}

std::pair<StatusCode, ObjectId>
FSSerializer::storeTree(std::shared_ptr<Tree> obj)
{
    ObjectId oid = mapit::hash_toString(obj.get());

    fs::path path = repo_ / _PREFIX_TREE_;
    fs_check_create( path );

    path /= fs::path(oid);

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    *(ge->mutable_tree()) = *obj;
    fs_write( path, ge, MessageTree);

    return std::pair<StatusCode, ObjectId>(MAPIT_STATUS_OK, oid);
}

std::pair<StatusCode, ObjectId>
FSSerializer::storeTreeTransient(std::shared_ptr<Tree> obj, const PathInternal &transientId)
{
    fs::path path = objectid_to_workspace_fs_path( transientId );
    fs_check_create( path );

    path /= _WORKSPACE_GENERIC_ENTRY_;

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    *(ge->mutable_tree()) = *obj;
    fs_write( path, ge, MessageTree, true);

    return std::pair<StatusCode, ObjectId>(MAPIT_STATUS_OK, transientId);
}

StatusCode
FSSerializer::removeTreeTransient(const PathInternal &transientId)
{
    fs::path path = objectid_to_workspace_fs_path( transientId ) / _WORKSPACE_GENERIC_ENTRY_;
    fs_delete( path );

    return MAPIT_STATUS_OK;
}

std::shared_ptr<Entity>
FSSerializer::getEntity(const ObjectId oid)
{
    fs::path path = repo_ / _PREFIX_ENTITY_ / fs::path(oid);

    if ( ! fs::exists( path ) ) {
        return std::shared_ptr<Entity>(NULL);
    }

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    fs_read(path, ge);

    if ( ge->type() != MessageEntity ) {
//        log_debug("Entity at: " + path.string() + " has variable type not set correctly");
        return nullptr;
    }
    if ( ! ge->has_entity() ) {
        log_fatal("Entity at: " + path.string() + " is no entity");
        return std::shared_ptr<Entity>(NULL);
    }

    return std::shared_ptr<Entity>(new Entity( ge->entity() ));
}

std::shared_ptr<Entity>
FSSerializer::getEntityTransient(const PathInternal oid)
{
    Path pathWithoutSlash = oid.substr(0, oid.length()- (oid[oid.length()-1] == '/')); //TODO is that realy needed ???

    fs::path path = objectid_to_workspace_fs_path(pathWithoutSlash) / _WORKSPACE_GENERIC_ENTRY_;

    if ( ! fs::exists( path ) ) {
        return std::shared_ptr<Entity>(NULL);
    }

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    fs_read(path, ge);

    if ( ge->type() != MessageEntity ) {
//        log_debug("Entity at: " + path.string() + " has variable type not set correctly");
        return nullptr;
    }
    if ( ! ge->has_entity() ) {
        log_fatal("Entity at: " + path.string() + " is no entity");
        return std::shared_ptr<Entity>(NULL);
    }

    return std::shared_ptr<Entity>(new Entity( ge->entity() ));
}

std::pair<StatusCode, ObjectId>
FSSerializer::storeEntity(std::shared_ptr<Entity> obj)
{
    ObjectId oid = mapit::hash_toString(obj.get());

    fs::path path = repo_ / _PREFIX_ENTITY_;
    fs_check_create( path );

    path /= fs::path(oid);

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    *(ge->mutable_entity()) = *obj;
    fs_write( path, ge, MessageEntity);

    return std::pair<StatusCode, ObjectId>(MAPIT_STATUS_OK, oid);
}

std::pair<StatusCode, ObjectId>
FSSerializer::storeEntityTransient(std::shared_ptr<Entity> obj, const PathInternal &transientId)
{
    fs::path path = objectid_to_workspace_fs_path( transientId );
    fs_check_create( path );

    path /= _WORKSPACE_GENERIC_ENTRY_;

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    *(ge->mutable_entity()) = *obj;
    fs_write( path, ge, MessageEntity, true);

    return std::pair<StatusCode, ObjectId>(MAPIT_STATUS_OK, transientId);
}

StatusCode
FSSerializer::removeEntityTransient(const PathInternal &transientId)
{
    // delete entity and entitydata
    fs::path path_entity = objectid_to_workspace_fs_path( transientId );
    fs_delete( path_entity / _WORKSPACE_GENERIC_ENTRY_ );
    fs_delete( path_entity / _workspace_ENTITY_DATA_ );

    return MAPIT_STATUS_OK;
}

std::shared_ptr<Commit>
FSSerializer::getCommit(const ObjectId &oid)
{
    fs::path path = repo_ / _PREFIX_COMMIT_ / fs::path(oid);

    if ( ! fs::exists( path ) ) {
        return std::shared_ptr<Commit>(NULL);
    }

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    fs_read(path, ge);

    if ( ge->type() != MessageCommit ) {
//        log_debug("Commit at: " + path.string() + " has variable type not set correctly");
        return nullptr;
    }
    if ( ! ge->has_commit() ) {
        log_fatal("Commit at: " + path.string() + " is no commit");
        return std::shared_ptr<Commit>(NULL);
    }

    return std::shared_ptr<Commit>(new Commit( ge->commit() ));
}

std::pair<StatusCode, ObjectId>
FSSerializer::createCommit(std::shared_ptr<Commit> obj)
{
    ObjectId oid = mapit::hash_toString(obj.get());

    fs::path path = repo_ / _PREFIX_COMMIT_;
    fs_check_create( path );

    path /= fs::path(oid);

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    *(ge->mutable_commit()) = *obj;
    fs_write( path, ge, MessageCommit);

    return std::pair<StatusCode, ObjectId>(MAPIT_STATUS_OK, oid);
}

StatusCode
FSSerializer::removeCommit(const ObjectId &oid)
{
    //TODO
    return MAPIT_STATUS_ERR_DB_IO_ERROR;
}

std::vector<std::string>
FSSerializer::listWorkspaceNames()
{
    fs::path workspaces = repo_ / _PREFIX_WORKSPACES_;
    std::vector<std::string> ret;
    if ( ! fs::exists(workspaces) ) {
        return ret;
    }

    fs::directory_iterator end_it;
    fs::directory_iterator it(workspaces);
    for (; it != end_it; ++it) {
        if ( fs::is_directory( it->status() ) ) {
            ret.push_back( it->path().filename().string() );
        }
    }

    return ret;
}

std::vector< std::shared_ptr<WorkspaceObj> >
FSSerializer::listWorkspaces()
{
    //TODO
    std::vector<std::shared_ptr<WorkspaceObj> > ret;
    return ret;
}

std::shared_ptr<WorkspaceObj>
FSSerializer::getworkspaceCommit(const std::string &name)
{
    fs::path path = objectid_to_workspace_fs_path(name) / _WORKSPACE_GENERIC_ENTRY_;

    if ( ! fs::exists( path ) ) {
        return std::shared_ptr<WorkspaceObj>(NULL);
    }

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    fs_read(path, ge);

    if ( ge->type() != MessageWorkspace ) {
//        log_error("Workspace at: " + path.string() + " has variable type not set correctly");
        return nullptr;
    }
    if ( ! ge->has_workspace() ) {
        log_fatal("Workspace at: " + path.string() + " is no workspace");
        return std::shared_ptr<WorkspaceObj>(NULL);
    }

    return std::shared_ptr<WorkspaceObj>(new WorkspaceObj( ge->workspace() ));
}

StatusCode
FSSerializer::storeWorkspaceCommit(std::shared_ptr<WorkspaceObj> obj, const std::string &name)
{
    fs::path path = objectid_to_workspace_fs_path( name );
    fs_check_create(path);

    path /= _WORKSPACE_GENERIC_ENTRY_;

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    *(ge->mutable_workspace()) = *obj;
    fs_write( path, ge, MessageWorkspace, true);

    return MAPIT_STATUS_OK;
}

StatusCode
FSSerializer::createworkspaceCommit(std::shared_ptr<WorkspaceObj> obj, const std::string &name)
{
    fs::path path = objectid_to_workspace_fs_path(name);
    fs_check_create( path );

    path /= _WORKSPACE_GENERIC_ENTRY_;

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    *(ge->mutable_workspace()) = *obj;
    fs_write( path, ge, MessageWorkspace);

    return MAPIT_STATUS_OK;
}

StatusCode
FSSerializer::removeWorkspace(const ObjectId &oid)
{
    fs::path path = objectid_to_workspace_fs_path( oid ) / _WORKSPACE_GENERIC_ENTRY_;
    fs_delete( path );

    return MAPIT_STATUS_OK;
}

std::vector< std::shared_ptr<Branch> >
FSSerializer::listBranches()
{
    fs::path branches = repo_ / _PREFIX_BRANCHES_;
    std::vector<std::shared_ptr<Branch>> ret;

    if ( ! fs::exists(branches) ) {
        return ret;
    }

    fs::recursive_directory_iterator end_it;
    fs::recursive_directory_iterator it(branches);
    for (; it != end_it; ++it) {
        if ( fs::is_regular_file( it->status() ) ) {
            std::shared_ptr<GenericEntry> ge(new GenericEntry);
            fs_read(it->path(), ge);

            bool branch_ok = true;
            if ( ge->type() != MessageBranch ) {
                log_error("Branch at: " + it->path().string() + " has variable type not set correctly");
                branch_ok = false;
            }
            if ( ! ge->has_branch() ) {
                log_fatal("Branch at: " + it->path().string() + " is no branch");
                branch_ok = false;
            }

            if ( branch_ok ) {
                std::shared_ptr<Branch> br( new Branch(ge->branch()) );
                ret.push_back( br );
            } else {
                log_error("Can't load branch \"" + it->path().string() + "\"");
            }
        }
    }

    return ret;
}

std::shared_ptr<Branch>
FSSerializer::getBranch(const std::string &name)
{
    fs::path path = repo_ / _PREFIX_BRANCHES_ / fs::path(name);

    if ( ! fs::exists( path ) ) {
        return std::shared_ptr<Branch>(NULL);
    }

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    fs_read(path, ge);

    if ( ge->type() != MessageBranch ) {
//        log_error("Branch at: " + path.string() + " has variable type not set correctly");
        return nullptr;
    }
    if ( ! ge->has_branch() ) {
        log_fatal("Branch at: " + path.string() + " is no branch");
        return std::shared_ptr<Branch>(NULL);
    }

    return std::shared_ptr<Branch>(new Branch( ge->branch() ));
}

StatusCode
FSSerializer::storeBranch(std::shared_ptr<Branch> obj, const std::string &name)
{
    //TODO
    return MAPIT_STATUS_ERR_DB_IO_ERROR;
}

StatusCode
FSSerializer::createBranch(std::shared_ptr<Branch> obj, const std::string &name)
{
    fs::path path = repo_ / _PREFIX_BRANCHES_;
    fs_check_create( path );
    path /= name;

    if (fs::exists( path )) {
        return MAPIT_STATUS_BRANCH_ALREADY_EXISTS;
    }

    std::shared_ptr<GenericEntry> ge(new GenericEntry);
    *(ge->mutable_branch()) = *obj;
    fs_write(path, ge, MessageBranch);

    return MAPIT_STATUS_OK;
}

StatusCode
FSSerializer::removeBranch(const std::string &name)
{
    //TODO
    return MAPIT_STATUS_ERR_DB_IO_ERROR;
}

bool
FSSerializer::existsStreamProvider(const ObjectId &entityId)
{
    fs::path path = repo_ / _PREFIX_ENTITY_ / fs::path(entityId);
    return fs::exists(path);
}

bool
FSSerializer::existsStreamProviderTransient(const Path &path)
{
    fs::path filePath = objectid_to_workspace_fs_path(path) / _workspace_ENTITY_DATA_;
    return fs::exists(filePath);
}

std::shared_ptr<AbstractEntitydataProvider>
FSSerializer::getStreamProvider(const ObjectId &entityId, bool canRead)
{
    //TODO: Get entity data by oid. Might be a file with name "entityId" in folder "_PREFIX_ENTITYDATA"?
    fs::path path = repo_ / _PREFIX_ENTITY_ / fs::path(entityId);
    std::string fn(path.string());
    return std::shared_ptr<AbstractEntitydataProvider>( new FileSystemEntitydataStreamProvider(canRead?fn:"", ""));
}

std::shared_ptr<AbstractEntitydataProvider>
FSSerializer::getStreamProviderTransient(const Path &oid, bool canRead, bool canWrite)
{
    //TODO: Get entity data by path. Might be a file at path "_PREFIX_CHECKOUT" / "path"?
    fs::path path = objectid_to_workspace_fs_path(oid) / _workspace_ENTITY_DATA_;
    std::string fn( path.string() );
    return std::shared_ptr<AbstractEntitydataProvider>( new FileSystemEntitydataStreamProvider(canRead?fn:"", canWrite?fn:""));
}

StatusCode
FSSerializer::cleanUp()
{
    //TODO
    return MAPIT_STATUS_ERR_DB_IO_ERROR;
}

MessageType
FSSerializer::typeOfObject(const ObjectId &oid)
{
    fs::path path;
    std::vector<fs::path> places;
    places.push_back(_PREFIX_COMMIT_);
    places.push_back(_PREFIX_BRANCHES_);
    places.push_back(_PREFIX_TREE_);
    places.push_back(_PREFIX_ENTITY_);
    places.push_back(_PREFIX_ENTITY_DATA_);

    bool found = false;
    for (std::vector<fs::path>::iterator it = places.begin();
         it != places.end();
         ++it) {
        path = repo_ / *it / fs::path( oid );

        if ( fs::exists( path ) ) {
            found = true;
            break;
        }
    }

    if ( ! found) {
        log_warn("Can't open file to detect type of object " + oid);
        return MessageEmpty;
    }

    std::shared_ptr<GenericEntry> ge (new GenericEntry);
    fs_read(path, ge);

    return ge->type();
}

MessageType
FSSerializer::typeOfObjectTransient(const PathInternal &pathIntenal)
{
    fs::path path = objectid_to_workspace_fs_path(pathIntenal) / _WORKSPACE_GENERIC_ENTRY_;

    if ( ! fs::exists( path) ) {
        log_warn("Can't open file to detect type of object " + path.string());
        return MessageEmpty;
    }

    std::shared_ptr<GenericEntry> ge (new GenericEntry);
    fs_read(path, ge);

    return ge->type();
}

bool
FSSerializer::exists(const ObjectId &oidOrName)
{
    //TODO
    return true;
}

std::pair<StatusCode, ObjectId>
FSSerializer::persistTransientEntitydata(const PathInternal &pathInternal)
{
    if (pathInternal.empty()) {
        return std::pair<StatusCode, ObjectId>(MAPIT_STATUS_OK, "");
    }

    // get EntityData path
    fs::path transientPath = objectid_to_workspace_fs_path(pathInternal) / _workspace_ENTITY_DATA_;

    // calculate chunk size
    size_t entitySize = fs::file_size(transientPath);
    size_t offsetMax = 4 * 1024 * 1024; // 4 MB // what makes sense? 100MB?
    size_t offsetStep = entitySize;
    if ( offsetStep > offsetMax ) {
        offsetStep = offsetMax;
    }

    // create buffer for hash TODO, use SHA update to not store in RAM
    std::string fileHash = "";

    // read chunks, to create the hash
    std::ifstream transientStream(transientPath.string(), std::ifstream::in | std::ios_base::binary);
    bool writeIsDone = false;
    for (size_t offsetIterator = 0; ! writeIsDone; offsetIterator++) {
        char* buffer = new char[offsetStep];

        if ( transientStream.read(buffer, offsetStep) ) { // if the end is not reached
            // and update hash
            fileHash += std::string(buffer, offsetStep); // TODO, use update of hash to be able to not have the whole file in RAM
        } else {
            // and update hash
            size_t left = transientStream.gcount();
            fileHash += std::string(buffer, left); // TODO, use update of hash to be able to not have the whole file in RAM

            writeIsDone = true; // stop writing with next loop
        }
        delete[] buffer;
    }
    transientStream.close();

    // finish hash
    ObjectId persistentID = hash_toString(fileHash);

    // move temporarly file to correct position with hash
    fs::path persistentPath = repo_ / _PREFIX_ENTITY_DATA_ / persistentID;
    fs_check_create( persistentPath.parent_path() );
    fs::rename(transientPath, persistentPath);

    return std::pair<StatusCode, ObjectId>(MAPIT_STATUS_OK, persistentID);
}

bool
FSSerializer::canRead()
{
    //TODO
    return true;
}

bool
FSSerializer::canWrite()
{
    //TODO
    return true;
}
#ifdef MAPIT_DEBUG
void
FSSerializer::debugDump()
{
    //TODO
}

#endif
}
