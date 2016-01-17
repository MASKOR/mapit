#include "leveldbserializer.h"
#include "upns.h"
#include "util.h"
#include "leveldbentitydatastreamprovider.h"
#include "leveldb/db.h"
#include <assert.h>
#include <services.pb.h>
#include <stdlib.h>
#include <time.h>
#include "error.h"
#include "../hash.h"

#include <QDebug>
#include <QDateTime>
#include <QFileInfo>
#include <QLockFile>

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

LevelDBSerializer::LevelDBSerializer(const YAML::Node &config)
{
    std::string databaseName;
    if(config["filename"])
    {
        databaseName = config["filename"].as<std::string>();
    }
    else
    {
        databaseName = "maps.db";
    }
    log_info("opening database " + databaseName);

    // check lockfile, database must only be opened once.
    // Note: leveldb seems to have locking on its own and does not allow simultaneous access at all
    QString databaseNameQStr = QString::fromStdString(databaseName);
    m_lockFile = new QLockFile(QFileInfo(databaseNameQStr).absolutePath() + "." + QFileInfo(databaseNameQStr).fileName() + ".lock");
    if(!m_lockFile->tryLock())
    {
        qint64 pid;
        QString hostname;
        QString appname;
        m_lockFile->getLockInfo( &pid, &hostname, &appname);
        log_error(std::string("database already open. Indicated by lockfile ") + QFileInfo(databaseNameQStr).canonicalFilePath().toStdString()
                  + "\n PID: " + QString::number(pid).toStdString()
                  + "\n Hostname: " + hostname.toStdString()
                  + "\n Application: " + appname.toStdString());
    }

    leveldb::Options options;
    options.create_if_missing = true;
    leveldb::Status status = leveldb::DB::Open(options, databaseName, &m_db);
    assert(status.ok());
}

LevelDBSerializer::~LevelDBSerializer()
{
    // destruction of m_lockFile will unlock the database if previously acquired.
    delete m_db;
    delete m_lockFile;
}

template <>
StatusCode LevelDBSerializer::createObject(const std::string &key, upnsSharedPointer<Tree> value)
{
    GenericEntry entry;
    entry.set_type(MessageTree);
    *entry.mutable_tree() = *value;
    return createObject(key, entry.SerializeAsString());
}
template <>
StatusCode LevelDBSerializer::createObject(const std::string &key, upnsSharedPointer<Entity> value)
{
    GenericEntry entry;
    entry.set_type(MessageEntity);
    *entry.mutable_entity() = *value;
    return createObject(key, entry.SerializeAsString());
}
template <>
StatusCode LevelDBSerializer::createObject(const std::string &key, upnsSharedPointer<Commit> value)
{
    GenericEntry entry;
    entry.set_type(MessageCommit);
    *entry.mutable_commit() = *value;
    return createObject(key, entry.SerializeAsString());
}
template <>
StatusCode LevelDBSerializer::createObject(const std::string &key, upnsSharedPointer<CheckoutObj> value)
{
    GenericEntry entry;
    entry.set_type(MessageCheckout);
    *entry.mutable_checkout() = *value;
    return createObject(key, entry.SerializeAsString());
}
template <>
StatusCode LevelDBSerializer::createObject(const std::string &key, upnsSharedPointer<Branch> value)
{
    GenericEntry entry;
    entry.set_type(MessageBranch);
    *entry.mutable_branch() = *value;
    return createObject(key, entry.SerializeAsString());
}
//template <typename T>
//StatusCode LevelDBSerializer::createObject(const std::string &key, upnsSharedPointer<bytes> value)
//{
//    GenericEntry entry;
//    entry.set_type(Message);
//    entry.set_allocated_(value.get());
//    return createObject(key, entry->SerializeAsString());
//}
//TODO: not needed anymore
//template <typename T>
//StatusCode LevelDBSerializer::createObject(const std::string &key, upnsSharedPointer<T> value)
//{
//    //(*iter)->set_lastchange(QDateTime::currentDateTime().toMSecsSinceEpoch());
//    //entity->set_id(generateId());
////    const ::google::protobuf::Descriptor* descriptor = value->GetDescriptor();
////    std::string ser = descriptor->name();
////    log_info("databaseval start:" + ser);
////    return createObject(key, ser + value->SerializeAsString());
//}

upnsSharedPointer<Tree> LevelDBSerializer::getTree(const ObjectId &oid)
{
    upnsSharedPointer<Tree> ret = getObject<Tree>( keyOfTree( oid ) );
    return ret;
}

StatusCode LevelDBSerializer::storeTree(upnsSharedPointer<Tree> &obj)
{
    std::string key = ::upns::hash_toString(obj.get());
    obj->set_id(key);
    key = keyOfTree(key);
    return storeObject(key, obj);
}

StatusCode LevelDBSerializer::createTree(upnsSharedPointer<Tree> &obj)
{
    std::string key = ::upns::hash_toString(obj.get());
    key = keyOfTree(key);
    return createObject(key, obj);
}

StatusCode LevelDBSerializer::removeTree(const ObjectId &oid)
{
    return removeObject(oid);
}

upnsSharedPointer<Entity> LevelDBSerializer::getEntity(const ObjectId oid)
{
    upnsSharedPointer<Entity> ret = getObject<Entity>( keyOfEntity( oid ) );
    return ret;
}

StatusCode LevelDBSerializer::storeEntity(upnsSharedPointer<Entity> &obj)
{
    std::string key = ::upns::hash_toString(obj.get());
    key = keyOfEntity(key);
    return storeObject(key, obj);
}

StatusCode LevelDBSerializer::createEntity(upnsSharedPointer<Entity> &obj)
{
    std::string key = ::upns::hash_toString(obj.get());
    key = keyOfEntity(key);
    return createObject(key, obj);
}

StatusCode LevelDBSerializer::removeEntity(const ObjectId &oid)
{
    return removeObject(oid);
}

upnsSharedPointer<Commit> LevelDBSerializer::getCommit(const ObjectId &oid)
{
    upnsSharedPointer<Commit> ret = getObject<Commit>( keyOfCommit( oid ) );
    return ret;
}

StatusCode LevelDBSerializer::storeCommit(upnsSharedPointer<Commit> &obj)
{
    std::string key = ::upns::hash_toString(obj.get());
    key = keyOfCommit(key);
    return createObject(key, obj);
}

StatusCode LevelDBSerializer::createCommit(upnsSharedPointer<Commit> &obj)
{
    std::string key = ::upns::hash_toString(obj.get());
    key = keyOfCommit(key);
    return createObject(key, obj);
}

StatusCode LevelDBSerializer::removeCommit(const ObjectId &oid)
{
    return removeObject(oid);
}

upnsVec<upnsString> LevelDBSerializer::listCheckoutNames()
{
    upnsVec<upnsString > ret;
    leveldb::Iterator* it = m_db->NewIterator(leveldb::ReadOptions());
    for (it->SeekToFirst(); it->Valid(); it->Next()) {
        const leveldb::Slice &key = it->key();
        if(key.starts_with(KEY_PREFIX_CHECKOUT))
        {
            const upnsString oid = sliceEndToId( key );
            ret.push_back( oid );
        }
    }
    assert(it->status().ok());  // Check for any errors found during the scan
    delete it;
    return ret;
}

upnsVec< upnsSharedPointer<CheckoutObj> > LevelDBSerializer::listCheckouts()
{
    upnsVec<upnsSharedPointer<CheckoutObj> > ret;
    leveldb::Iterator* it = m_db->NewIterator(leveldb::ReadOptions());
    for (it->SeekToFirst(); it->Valid(); it->Next()) {
        const leveldb::Slice &key = it->key();
        if(key.starts_with(KEY_PREFIX_CHECKOUT))
        {
            upnsSharedPointer<CheckoutObj> co( new CheckoutObj() );
            co->ParseFromString(it->value().data());
            ret.push_back( co );
        }
    }
    assert(it->status().ok());  // Check for any errors found during the scan
    delete it;
    return ret;
}

upnsSharedPointer<CheckoutObj> LevelDBSerializer::getCheckoutCommit(const upnsString &name)
{
    upnsSharedPointer<CheckoutObj> ret = getObject<CheckoutObj>( keyOfCheckoutCommit( name ) );
    return ret;
}

StatusCode LevelDBSerializer::storeCheckoutCommit(upnsSharedPointer<CheckoutObj> &obj, const upnsString &name)
{
    std::string key = keyOfCheckoutCommit(name);
    return createObject(key, obj);
}

StatusCode LevelDBSerializer::createCheckoutCommit(upnsSharedPointer<CheckoutObj> &obj, const upnsString &name)
{
    std::string key = keyOfCheckoutCommit(name);
    return createObject(key, obj);
}

StatusCode LevelDBSerializer::removeCheckoutCommit(const ObjectId &oid)
{
    return removeObject(oid);
}

upnsVec< upnsSharedPointer<Branch> > LevelDBSerializer::listBranches()
{
    upnsVec<upnsSharedPointer<Branch> > ret;
    leveldb::Iterator* it = m_db->NewIterator(leveldb::ReadOptions());
    GenericEntry entry;
    for (it->SeekToFirst(); it->Valid(); it->Next()) {
        const leveldb::Slice &key = it->key();
        if(key.starts_with(KEY_PREFIX_BRANCH))
        {
            StatusCode s = getGenericEntry(key, entry);
            if(!upnsIsOk( s ))
            {
                log_error("Parse error while listing branches");
                continue;
            }
            if(entry.type() == MessageBranch)
            {
                // copy branch and give shared pointer ownership
                upnsSharedPointer<Branch> br( new Branch(entry.branch()) );
                ret.push_back( br );
            }
        }
    }
    assert(it->status().ok());  // Check for any errors found during the scan
    delete it;
    return ret;
}

upnsSharedPointer<Branch> LevelDBSerializer::getBranch(const upnsString &name)
{
    upnsSharedPointer<Branch> ret = getObject<Branch>( keyOfBranch( name ) );
    return ret;
}

StatusCode LevelDBSerializer::storeBranch(upnsSharedPointer<Branch> &obj, const upnsString &name)
{
    std::string key = keyOfBranch(name);
    return createObject(key, obj);
}

StatusCode LevelDBSerializer::createBranch(upnsSharedPointer<Branch> &obj, const upnsString &name)
{
    std::string key = keyOfBranch(name);
    return createObject(key, obj);
}

StatusCode LevelDBSerializer::removeBranch(const upnsString &name)
{
    return removeObject(name);
}

std::string LevelDBSerializer::keyOfTree(const ObjectId &oid) const
{
    std::stringstream strstr;
    strstr << KEY_PREFIX_TREE << LDBSER_DELIM << oid;
    return strstr.str();
}

std::string LevelDBSerializer::keyOfEntity(const ObjectId &oid) const
{
    std::stringstream strstr;
    strstr << KEY_PREFIX_ENTITY << LDBSER_DELIM << oid;
    return strstr.str();
}

std::string LevelDBSerializer::keyOfEntityData(const ObjectId &oid) const
{
    std::stringstream strstr;
    strstr << KEY_PREFIX_DATA << LDBSER_DELIM << oid;
    return strstr.str();
}

std::string LevelDBSerializer::keyOfCommit(const ObjectId &oid) const
{
    std::stringstream strstr;
    strstr << KEY_PREFIX_COMMIT << LDBSER_DELIM << oid;
    return strstr.str();
}

std::string LevelDBSerializer::keyOfCheckoutCommit(const upnsString &name) const
{
    std::stringstream strstr;
    strstr << KEY_PREFIX_CHECKOUT << LDBSER_DELIM << name; //<< oid;
    return strstr.str();
}

std::string LevelDBSerializer::keyOfBranch(const upnsString &name) const
{
    std::stringstream strstr;
    strstr << KEY_PREFIX_BRANCH << LDBSER_DELIM << name; //<< oid;
    return strstr.str();
}

upnsSharedPointer<AbstractEntityDataStreamProvider> LevelDBSerializer::getStreamProvider(const ObjectId &entityId, bool canRead, bool canWrite)
{
    //TODO: ensure readOnly and store booleans
    return upnsSharedPointer<AbstractEntityDataStreamProvider>( new LevelDBEntityDataStreamProvider(m_db, keyOfEntityData(entityId)));
}

StatusCode LevelDBSerializer::cleanUp()
{
    return 0;
}

MessageType LevelDBSerializer::typeOfObject(const ObjectId &oidOrName)
{
    //TODO
    return MessageBranch;
}

bool LevelDBSerializer::exists(const ObjectId &oidOrName)
{
    std::string key, value;
    std::string (LevelDBSerializer::* keyOfMethods [])(const ObjectId &) const = {
            &LevelDBSerializer::keyOfTree,
            &LevelDBSerializer::keyOfEntity,
            //TODO: How will EntityData behave?
            &LevelDBSerializer::keyOfEntityData,
            &LevelDBSerializer::keyOfBranch,
            &LevelDBSerializer::keyOfCommit,
            &LevelDBSerializer::keyOfCheckoutCommit};
    for(int i=0; i < sizeof(keyOfMethods)/sizeof(*keyOfMethods) ; ++i)
    {
        key = (*this.*(keyOfMethods[i]))(oidOrName);
        leveldb::Status s = m_db->Get(leveldb::ReadOptions(), key, &value);
        if(s.ok()) return true;
    }
    return false;
}
//bool LevelDBSerializer::isTree(const ObjectId &oid)
//{
////    GenericEntry e;
////    getGenericEntry(keyOfTree(oid),
//}

//bool LevelDBSerializer::isEntity(const ObjectId &oid)
//{

//}

//bool LevelDBSerializer::isCommit(const CommitId &oid)
//{

//}

//bool LevelDBSerializer::isCheckout(const CommitId &oid)
//{

//}

//bool LevelDBSerializer::isBranch(const CommitId &oid)
//{

//}

bool LevelDBSerializer::canRead()
{
    return true;
}

bool LevelDBSerializer::canWrite()
{
    return true;
}

StatusCode LevelDBSerializer::levelDbStatusToUpnsStatus(const leveldb::Status &levelDbStatus)
{
    if(levelDbStatus.ok()) {
        return UPNS_STATUS_OK;
    } else if(levelDbStatus.IsNotFound()) {
        return UPNS_STATUS_ERR_DB_NOT_FOUND;
    } else if(levelDbStatus.IsCorruption()) {
        return UPNS_STATUS_ERR_DB_CORRUPTION;
    } else if(levelDbStatus.IsIOError()) {
        return UPNS_STATUS_ERR_DB_IO_ERROR;
    } else {
        // might be...
        // UPNS_STATUS_ERR_DB_NOT_SUPPORTED;
        // UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
        return UPNS_STATUS_ERR_DB_UNKNOWN;
    }
}

template <typename T>
upnsSharedPointer< T > LevelDBSerializer::getObject(const std::string &key)
{
    GenericEntry value;
    StatusCode s = getGenericEntry(key, value);
    if(!upnsIsOk( s )) return NULL;

    upnsSharedPointer< T > ret = fromGeneric<T>(value);
    return ret;
//    if(!ret)
//    {
//        //do not log here because it is okay to have null
//        //log_error("could not parse payload: " + key);
//        return NULL;
//    }
//    return ret;
}

StatusCode LevelDBSerializer::getObject(const std::string &key, std::string &value)
{
    return getObject(leveldb::Slice(key), value);
}

StatusCode LevelDBSerializer::getObject(const leveldb::Slice &key, std::string &value)
{
    // do not log here, fails might be wanted. (leveldb has no "exists" method)
    leveldb::Status s = m_db->Get(leveldb::ReadOptions(), key, &value);
    if(!s.ok())
    {
        if(s.IsNotFound())
        {
            //log_error("could not find map with id: " + key.data());
            return UPNS_STATUS_ERR_DB_NOT_FOUND;
        }
        else if(s.IsIOError())
        {
            //log_error("IO Error for map with id: " + key.data());
            return UPNS_STATUS_ERR_DB_IO_ERROR;
        }
        else if(s.IsCorruption())
        {
            //log_error("Corruption Error for map with id: " + key.data());
            return UPNS_STATUS_ERR_DB_CORRUPTION;
        }
        return UPNS_STATUS_ERR_DB_UNKNOWN;
     }
     return UPNS_STATUS_OK;
}

StatusCode LevelDBSerializer::getGenericEntryFromOid(const ObjectId &oidOrName, GenericEntry &value)
{
    // Try all "syntaxes" of keys.
    // This is not a nice solution. However, different key syntaxes are used to distinguish object types from each other and generate lists 'listCheckouts'.
    // An alternative approach would store all objects with the same key and traverse though all objects and filter them in the 'list*' methods..
    // TODO: Think about using common key structure and filter as said above. Delete this hacky loop.
    //       Maybe two different keytypes are needed for names and hashes
    StatusCode s;
    std::string key;
    std::string (LevelDBSerializer::* keyOfMethods [])(const ObjectId &) const = {
            &LevelDBSerializer::keyOfTree,
            &LevelDBSerializer::keyOfEntity,
            //TODO: How will EntityData behave?
            &LevelDBSerializer::keyOfEntityData,
            &LevelDBSerializer::keyOfBranch,
            &LevelDBSerializer::keyOfCommit,
            &LevelDBSerializer::keyOfCheckoutCommit};
    for(int i=0; i < sizeof(keyOfMethods)/sizeof(*keyOfMethods) ; ++i)
    {
        key = (*this.*(keyOfMethods[i]))(oidOrName);
        s = getGenericEntry(key, value);
        // Break if entry was found or anything except "not found" happened.
        // Note: If two types of objects share the same key-prefix this might be broken!
        if(upnsIsOk(s) || s != UPNS_STATUS_ENTITY_NOT_FOUND) return s;
    }
    return UPNS_STATUS_ENTITY_NOT_FOUND;
}

StatusCode LevelDBSerializer::getGenericEntry(const std::string &key, GenericEntry &value)
{
    return getGenericEntry(leveldb::Slice(key), value);
}

StatusCode LevelDBSerializer::getGenericEntry(const leveldb::Slice &key, GenericEntry &value)
{
    std::string str;
    StatusCode s = getObject(key, str);
    if(!upnsIsOk( s )) return UPNS_STATUS_ERR_DB_NOT_FOUND;
    if(!value.ParseFromString( str ))
    {
        log_error("could not parse generic entry for: " + key.data());
        return UPNS_STATUS_ERR_DB_PARSE;
    }
    return UPNS_STATUS_OK;
}

StatusCode LevelDBSerializer::storeObject(const std::string &key, const std::string &value)
{
    if( key.empty() )
    {
        log_error("tried to store object with id 0");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    // Get the previously stored map to check for correct version (optimistic locking)
    // Note: leveldb has no transactions. It is not guaranteed, that stuff is done between the next "Get" and "Put".
    // A guarantee can be given, if only one thread is used to access the database.
    std::string buff;
    leveldb::Status s = m_db->Get(leveldb::ReadOptions(), key, &buff);
    if(s.ok())
    {
        log_warn("tried to write existing object (object with same hash already stored): " + key);
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    if( !s.IsNotFound() )
    {
        log_warn("Error while storing object " + key);
    }
    else
    {
        // Not Found! Everything ok here, value has new key/hash
        s = m_db->Put(leveldb::WriteOptions(), key, value);
    }
    return levelDbStatusToUpnsStatus(s);
}

StatusCode LevelDBSerializer::removeObject(const std::string &oid)
{
    return 0;
}

StatusCode LevelDBSerializer::createObject(const std::string &key, const std::string &value)
{
    if( key.empty() )
    {
        log_error("tried to create object with id 0");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    // Get the previously stored map to check for correct version (optimistic locking)
    // Note: leveldb has no transactions. It is not guaranteed, that stuff is done between the next "Get" and "Put".
    // A guarantee can be given, if only one thread is used to access the database.
    std::string buff;
    leveldb::Status s = m_db->Get(leveldb::ReadOptions(), key, &buff);
    if(s.ok())
    {
        log_warn("tried to create existing object (object with same hash already stored): " + key);
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    if( !s.IsNotFound() )
    {
        log_warn("Error while storing object " + key);
    }
    else
    {
        // Not Found! Everything ok here, value has new key/hash
        s = m_db->Put(leveldb::WriteOptions(), key, value);
    }
    return levelDbStatusToUpnsStatus(s);
}

template <typename T>
StatusCode LevelDBSerializer::storeObject(const std::string &key, upnsSharedPointer<T> value)
{
    return storeObject(key, value->SerializeAsString());
}

template <>
upnsSharedPointer<Tree> LevelDBSerializer::fromGeneric(const GenericEntry &from)
{
    if(from.type() != MessageTree) return NULL;
    return upnsSharedPointer< Tree >(new Tree(from.tree()));
}
template <>
upnsSharedPointer<Entity> LevelDBSerializer::fromGeneric(const GenericEntry &from)
{
    if(from.type() != MessageEntity) return NULL;
    return upnsSharedPointer< Entity >(new Entity(from.entity()));
}
template <>
upnsSharedPointer<Commit> LevelDBSerializer::fromGeneric(const GenericEntry &from)
{
    if(from.type() != MessageCommit) return NULL;
    return upnsSharedPointer< Commit >(new Commit(from.commit()));
}
template <>
upnsSharedPointer<CheckoutObj> LevelDBSerializer::fromGeneric(const GenericEntry &from)
{
    if(from.type() != MessageCheckout) return NULL;
    return upnsSharedPointer< CheckoutObj >(new CheckoutObj(from.checkout()));
}
template <>
upnsSharedPointer<Branch> LevelDBSerializer::fromGeneric(const GenericEntry &from)
{
    if(from.type() != MessageBranch) return NULL;
    return upnsSharedPointer< Branch >(new Branch(from.branch()));
}
//template <>
//upnsSharedPointer<bytes> LevelDBSerializer::fromGeneric(const GenericEntry &from)
//{
//    if(from.type() != MessageCheckout) return NULL;
//    return upnsSharedPointer< CheckoutObj >(new CheckoutObj(from.tree()));
//}

}
