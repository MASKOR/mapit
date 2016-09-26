#include "fs_serializer.h"
#include "upns.h"
#include "util.h"
#include "../hash.h"
#include "fs_entitydatastreamprovider.h"
#include <assert.h>
#include <services.pb.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include "error.h"

#define LDBSER_DELIM "!"
#define KEY_PREFIX_CHECKOUT "co"
#define KEY_PREFIX_COMMIT "obj!cm"
#define KEY_PREFIX_TREE "obj!tr"
#define KEY_PREFIX_ENTITY "obj!ent"
#define KEY_PREFIX_DATA "blob!ed"
#define KEY_PREFIX_TAG "obj!tag"
#define KEY_PREFIX_BRANCH "ref!branch"

namespace upns
{
const std::string FSSerializer::_PREFIX_MAPIT_      = "/.mapit";
const std::string FSSerializer::_PREFIX_TREE_       = "/trees/";
const std::string FSSerializer::_PREFIX_ENTITY_     = "/entities/";
const std::string FSSerializer::_PREFIX_CHECKOUTS_  = "/checkouts/";
const std::string FSSerializer::_PREFIX_BRANCHES_   = "/branches/";
const std::string FSSerializer::_PREFIX_COMMIT_     = "/commits/";

const std::string FSSerializer::_FILE_CHECKOUT_     = "checkout";

FSSerializer::FSSerializer(const YAML::Node &config)
{
    std::string cfgpath;
    if(config["filename"])
    {
        cfgpath = config["filename"].as<std::string>();
    }
    else
    {
        cfgpath = fs::current_path().string();
    }
    fs::path path(cfgpath);

    // search for repo
    while ( ! fs::exists( fs::system_complete( path.string() + _PREFIX_MAPIT_ ) ) ) {
        path = path.parent_path();

        if ( ! fs::exists( path ) ) {
            log_warn("Could not find " + _PREFIX_MAPIT_ + " in this or its parent folders");
            path = fs::current_path();
            log_warn("Create " + path.string() + _PREFIX_MAPIT_);
            if ( ! fs::create_directory( path.string() + _PREFIX_MAPIT_)) {
                log_error("Could not create " + path.string() + _PREFIX_MAPIT_);
            }
            break;
        }
    }
    path = path.string() + _PREFIX_MAPIT_;
    log_info("Use " + path.string() + " as repo");
    repo_ = path;

    // TODO do I need to lock this repo?
}

FSSerializer::~FSSerializer()
{

}

void
FSSerializer::fs_check_create(fs::path path)
{
    if ( ! fs::exists( path ) ) {
        log_info("Create: " + path.string());
        fs::create_directories( path );
    }
}

void
FSSerializer::fs_write(fs::path path, std::string value)
{
    if ( ! fs::exists( path ) ) {
        log_info("Write " + path.string());
        create_directories(path.parent_path());

        std::filebuf buffer;
        buffer.open(path.string(), std::ios::out);

        std::ostream stream(&buffer);
        stream << value;

        buffer.close();
    } else {
        log_info("Duplicate found " + path.string());
    }
}

void
FSSerializer::fs_read(fs::path path, upnsSharedPointer<GenericEntry> entry)
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

upnsSharedPointer<Tree>
FSSerializer::getTree(const ObjectId &oid)
{
    fs::path path( repo_ / fs::path( _PREFIX_CHECKOUTS_ ) / oid);

    if ( ! fs::exists( path ) ) {
        return upnsSharedPointer<Tree>(NULL);
    }

    upnsSharedPointer<GenericEntry> entry(new GenericEntry);
    fs_read(path, entry);

    return upnsSharedPointer<Tree>(new Tree( entry->tree() ));
}

upnsSharedPointer<Tree>
FSSerializer::getTreeTransient(const ObjectId &transientId)
{
    fs::path path = repo_ / fs::path( _PREFIX_CHECKOUTS_ ) /fs::path( transientId ) / fs::path(".tree");

    upnsSharedPointer<GenericEntry> entry(new GenericEntry);
    fs_read(path, entry);

    return upnsSharedPointer<Tree>(new Tree( entry->tree() ));
}

upnsPair<StatusCode, ObjectId>
FSSerializer::storeTree(upnsSharedPointer<Tree> &obj)
{
    ObjectId oid = upns::hash_toString(obj.get());

    fs::path path = repo_ / _PREFIX_TREE_;
    fs_check_create( path );

    fs_write( repo_ / _PREFIX_TREE_ / fs::path(oid), obj->SerializeAsString());

    StatusCode s = UPNS_STATUS_OK;
    return upnsPair<StatusCode, ObjectId>(s, oid);
}

upnsPair<StatusCode, ObjectId>
FSSerializer::storeTreeTransient(upnsSharedPointer<Tree> &obj, const ObjectId &transientId)
{
    fs::path path = repo_ / fs::path( _PREFIX_CHECKOUTS_ ) / fs::path( transientId );
    fs_check_create( path );

    path /= fs::path(".tree");
    GenericEntry entr;
    *entr.mutable_tree() = *obj;
    entr.set_type(MessageTree);
    fs_write( path, entr.SerializeAsString());

    return upnsPair<StatusCode, ObjectId>(UPNS_STATUS_OK, transientId);
}

StatusCode
FSSerializer::removeTree(const ObjectId &oid)
{
    //TODO
    return UPNS_STATUS_ERR_DB_IO_ERROR;
}

upnsSharedPointer<Entity>
FSSerializer::getEntity(const ObjectId oid)
{
    fs::path path(_PREFIX_ENTITY_ + oid);

    if ( ! fs::exists( path ) ) {
        return upnsSharedPointer<Entity>(NULL);
    }

    upnsSharedPointer<GenericEntry> entry(new GenericEntry);
    fs_read(path, entry);

    return upnsSharedPointer<Entity>(new Entity( entry->entity() ));
}

upnsSharedPointer<Entity>
FSSerializer::getEntityTransient(const Path path)
{
    //TODO
    Path pathWithoutSlash = path.substr(0, path.length()- (path[path.length()-1] == '/'));

    fs::path p = repo_ / fs::path(_PREFIX_CHECKOUTS_) / fs::path(pathWithoutSlash);

    if ( ! fs::exists( p.parent_path() ) ) {
        return upnsSharedPointer<Entity>(NULL);
    }

    upnsSharedPointer<GenericEntry> entry(new GenericEntry);
    fs_read(p, entry);

    return upnsSharedPointer<Entity>(new Entity( entry->entity() ));
}

upnsPair<StatusCode, ObjectId>
FSSerializer::storeEntity(upnsSharedPointer<Entity> &obj)
{
    ObjectId oid = upns::hash_toString(obj.get());

    fs::path path = repo_ / _PREFIX_ENTITY_;
    fs_check_create( path );

    path /= fs::path(oid);
    GenericEntry entr;
    *entr.mutable_entity() = *obj;
    entr.set_type(MessageEntity);
    fs_write( path, entr.SerializeAsString());

    return upnsPair<StatusCode, ObjectId>(UPNS_STATUS_OK, oid);
}

upnsPair<StatusCode, ObjectId>
FSSerializer::storeEntityTransient(upnsSharedPointer<Entity> &obj, const ObjectId &transientId)
{
    fs::path path = repo_ / fs::path( _PREFIX_CHECKOUTS_ ) / fs::path( transientId );

    GenericEntry entr;
    *entr.mutable_entity() = *obj;
    entr.set_type(MessageEntity);
    fs_write( path, entr.SerializeAsString());

    return upnsPair<StatusCode, ObjectId>(UPNS_STATUS_OK, transientId);
}

StatusCode
FSSerializer::removeEntity(const ObjectId &oid)
{
    //TODO
    return UPNS_STATUS_ERR_DB_IO_ERROR;
}

upnsSharedPointer<Commit>
FSSerializer::getCommit(const ObjectId &oid)
{
    fs::path path(_PREFIX_COMMIT_ + oid);

    if ( ! fs::exists( path ) ) {
        return upnsSharedPointer<Commit>(NULL);
    }

    upnsSharedPointer<GenericEntry> entry(new GenericEntry);
    fs_read(path, entry);

    return upnsSharedPointer<Commit>(new Commit( entry->commit() ));
}

upnsPair<StatusCode, ObjectId>
FSSerializer::createCommit(upnsSharedPointer<Commit> &obj)
{
    //TODO
    ObjectId oid;
    return upnsPair<StatusCode, ObjectId>(UPNS_STATUS_ERR_DB_IO_ERROR, oid);
}

StatusCode
FSSerializer::removeCommit(const ObjectId &oid)
{
    //TODO
    return UPNS_STATUS_ERR_DB_IO_ERROR;
}

upnsVec<upnsString>
FSSerializer::listCheckoutNames()
{
    //TODO
    upnsVec<upnsString > ret;
    return ret;
}

upnsVec< upnsSharedPointer<CheckoutObj> >
FSSerializer::listCheckouts()
{
    //TODO
    upnsVec<upnsSharedPointer<CheckoutObj> > ret;
    return ret;
}

upnsSharedPointer<CheckoutObj>
FSSerializer::getCheckoutCommit(const upnsString &name)
{
    fs::path path(fs::path(repo_) / fs::path(_PREFIX_CHECKOUTS_) / fs::path(name) / fs::path(_FILE_CHECKOUT_));

    if ( ! fs::exists( path ) ) {
        return upnsSharedPointer<CheckoutObj>(NULL);
    }

    upnsSharedPointer<GenericEntry> entry(new GenericEntry);
    fs_read(path, entry);

    return upnsSharedPointer<CheckoutObj>(new CheckoutObj( entry->checkout() ));
}

StatusCode
FSSerializer::storeCheckoutCommit(upnsSharedPointer<CheckoutObj> &obj, const upnsString &name)
{
    fs::path path = repo_ / fs::path( _PREFIX_CHECKOUTS_) / fs::path( name );

    fs_check_create(path);

    path /= fs::path( _FILE_CHECKOUT_ );

    GenericEntry entr;
    *entr.mutable_checkout() = *obj;
    entr.set_type(MessageCheckout);
    fs_write( path, entr.SerializeAsString());

    return UPNS_STATUS_OK;
}

StatusCode
FSSerializer::createCheckoutCommit(upnsSharedPointer<CheckoutObj> &obj, const upnsString &name)
{
    fs::path path = repo_ / fs::path(_PREFIX_CHECKOUTS_);
    fs_check_create( path );
    path /= name;
    fs_check_create( path );
    path /= _FILE_CHECKOUT_;
    GenericEntry entr;
    *entr.mutable_checkout() = *obj;
    entr.set_type(MessageCheckout);
    fs_write( path, entr.SerializeAsString());

    return UPNS_STATUS_OK;
}

StatusCode
FSSerializer::removeCheckoutCommit(const ObjectId &oid)
{
    //TODO
    return UPNS_STATUS_ERR_DB_IO_ERROR;
}

upnsVec< upnsSharedPointer<Branch> >
FSSerializer::listBranches()
{
    //TODO
    upnsVec<upnsSharedPointer<Branch> > ret;
    return ret;
}

upnsSharedPointer<Branch>
FSSerializer::getBranch(const upnsString &name)
{
    fs::path path = repo_ / fs::path(_PREFIX_BRANCHES_) / fs::path(name);

    if ( ! fs::exists( path ) ) {
        return upnsSharedPointer<Branch>(NULL);
    }

    upnsSharedPointer<GenericEntry> entry(new GenericEntry);
    fs_read(path, entry);

    return upnsSharedPointer<Branch>(new Branch( entry->branch() ));
}

StatusCode
FSSerializer::storeBranch(upnsSharedPointer<Branch> &obj, const upnsString &name)
{
    //TODO
    return UPNS_STATUS_ERR_DB_IO_ERROR;
}

StatusCode
FSSerializer::createBranch(upnsSharedPointer<Branch> &obj, const upnsString &name)
{
    fs::path path = repo_ / fs::path(_PREFIX_BRANCHES_);
    fs_check_create( path );
    path /= name;

    if (fs::exists( path )) {
        return UPNS_STATUS_BRANCH_ALREADY_EXISTS;
    }

    fs_write(path, obj->SerializeAsString());

    return UPNS_STATUS_OK;
}

StatusCode
FSSerializer::removeBranch(const upnsString &name)
{
    //TODO
    return UPNS_STATUS_ERR_DB_IO_ERROR;
}

upnsSharedPointer<AbstractEntityDataStreamProvider>
FSSerializer::getStreamProvider(const ObjectId &entityId, bool canRead)
{
    //TODO: Get entity data by oid. Might be a file with name "entityId" in folder "_PREFIX_ENTITYDATA"?
    fs::path p = repo_ / fs::path(_PREFIX_ENTITY_) / entityId;
    std::string fn(p.string());
    return upnsSharedPointer<AbstractEntityDataStreamProvider>( new FileSystemEntityDataStreamProvider(fn, fn));
}

upnsSharedPointer<AbstractEntityDataStreamProvider>
FSSerializer::getStreamProviderTransient(const Path &path, bool canRead, bool canWrite)
{
    //TODO: Get entity data by path. Might be a file at path "_PREFIX_CHECKOUT" / "path"?
    fs::path p = repo_ / fs::path(_PREFIX_CHECKOUTS_) / path;
    std::string fn(p.string() + ".fs_edata");
    return upnsSharedPointer<AbstractEntityDataStreamProvider>( new FileSystemEntityDataStreamProvider(fn, fn));
}

StatusCode
FSSerializer::cleanUp()
{
    //TODO
    return UPNS_STATUS_ERR_DB_IO_ERROR;
}

MessageType
FSSerializer::typeOfObject(const ObjectId &oidOrName)
{
    //TODO
    return MessageEmpty;
}

bool
FSSerializer::exists(const ObjectId &oidOrName)
{
    //TODO
    return true;
}

upnsPair<StatusCode, ObjectId>
FSSerializer::persistTransientEntityData(const ObjectId &entityId)
{
    //TODO
    return upnsPair<StatusCode, ObjectId>(UPNS_STATUS_ERR_DB_IO_ERROR, "");
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
#ifdef UPNS_DEBUG
void
FSSerializer::debugDump()
{
    //TODO
}

#endif
}
