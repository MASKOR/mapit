#include "fileserializer.h"
#include <iostream>
#include <fstream>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "libs/layertypes_collection/pointcloud2/src/pointcloudhelper.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

namespace upns
{
struct membuf: std::streambuf {
    membuf(char* base, std::ptrdiff_t n) {
        this->setg(base, base, base + n);
    }
};
class FileEntityDataStreamProvider : public AbstractEntityDataStreamProvider
{
    upnsString m_filename;
public:
    FileEntityDataStreamProvider(const upnsString &filename)
        :m_filename(filename)
    { }
    bool isCached() { return true; }
    bool isReadWriteSame() { return true; }
    upnsIStream *startRead(upnsuint64 start, upnsuint64 len)
    {
        //Note: Usually this is not the place meant to contain pointcloud logic.
        pcl::PCLPointCloud2 pc2;

        if(m_filename.substr(m_filename.length()-4, std::string::npos) == ".pcd")
        {
            pcl::PCDReader reader;
            if ( reader.read(m_filename, pc2) < 0 )
            {
                log_error("Couldn't read file" + m_filename);
                return NULL;
            }
        }
        else
        {
            pcl::PLYReader reader;
            if ( reader.read(m_filename, pc2) < 0 )
            {
                log_error("Couldn't read file" + m_filename);
                return NULL;
            }
        }
        std::iostream *is = new std::stringstream();
        ::boost::archive::text_oarchive oa(*is);
        oa << pc2;
        return is;
    }
    void endRead(upnsIStream *strm)
    {
        //std::ifstream *is = static_cast<std::ifstream*>(strm);
        //is->close();
        delete strm;
    }
    upnsOStream *startWrite(upnsuint64 start, upnsuint64 len)
    {
        std::ofstream *os = new std::ofstream(m_filename.c_str(), std::ifstream::binary);
        return os;
    }
    void endWrite(upnsOStream *strm)
    {
        std::ofstream *os = static_cast<std::ofstream*>(strm);
        os->close();
        delete os;
    }
    upnsuint64 getStreamSize() const
    {
        std::fstream s(m_filename.c_str(), std::fstream::binary);
        s.seekg (0, s.end);
        int length = s.tellg();
        s.seekg (0, s.beg);
        return length;
    }
    void setStreamSize(upnsuint64)
    {
        assert(true);
    }
    LockHandle lock()
    {
        return 0;
    }
    void unlock(LockHandle)
    {
    }
};

bool FileSerializer::canRead()
{
    return true;
}

bool FileSerializer::canWrite()
{
    //TODO: will never work or not yet implemented?
    return false;
}

upnsSharedPointer<Tree> FileSerializer::getTree(const ObjectId &oid)
{
    return NULL;
}

StatusCode FileSerializer::storeTree(upnsSharedPointer<Tree> &obj)
{
    return 0;
}

StatusCode FileSerializer::createTree(upnsSharedPointer<Tree> &obj)
{
    return 0;
}

StatusCode FileSerializer::removeTree(const ObjectId &oid)
{
    return 0;
}

upnsSharedPointer<Entity> FileSerializer::getEntity(const ObjectId oid)
{
    upnsSharedPointer<Entity> dummyPointcloud2Entity(new Entity());
    dummyPointcloud2Entity->set_type(LayerType::POINTCLOUD2);
    return dummyPointcloud2Entity;
}

StatusCode FileSerializer::storeEntity(upnsSharedPointer<Entity> &obj)
{
    return 0;
}

StatusCode FileSerializer::createEntity(upnsSharedPointer<Entity> &obj)
{
    return 0;
}

StatusCode FileSerializer::removeEntity(const ObjectId &oid)
{
    return 0;
}

upnsSharedPointer<Commit> FileSerializer::getCommit(const ObjectId &oid)
{
    return NULL;
}

StatusCode FileSerializer::storeCommit(upnsSharedPointer<Commit> &obj)
{
    return 0;
}

StatusCode FileSerializer::createCommit(upnsSharedPointer<Commit> &obj)
{
    return 0;
}

StatusCode FileSerializer::removeCommit(const ObjectId &oid)
{
    return 0;
}

upnsVec<upnsString> FileSerializer::listCheckoutNames()
{
    return upnsVec<upnsString>();
}

upnsVec<upnsSharedPointer<CheckoutObj> > FileSerializer::listCheckouts()
{
    return upnsVec<upnsSharedPointer<CheckoutObj> >();
}

upnsSharedPointer<CheckoutObj> FileSerializer::getCheckoutCommit(const upnsString &name)
{
    return NULL;
}

StatusCode FileSerializer::storeCheckoutCommit(upnsSharedPointer<CheckoutObj> &obj, const upnsString &name)
{
    return 0;
}

StatusCode FileSerializer::createCheckoutCommit(upnsSharedPointer<CheckoutObj> &obj, const upnsString &name)
{
    return 0;
}

StatusCode FileSerializer::removeCheckoutCommit(const upnsString &name)
{
    return 0;
}

upnsVec<upnsSharedPointer<Branch> > FileSerializer::listBranches()
{
    return upnsVec<upnsSharedPointer<Branch> >();
}

upnsSharedPointer<Branch> FileSerializer::getBranch(const upnsString &name)
{
    return NULL;
}

StatusCode FileSerializer::storeBranch(upnsSharedPointer<Branch> &obj, const upnsString &name)
{
    return 0;
}

StatusCode FileSerializer::createBranch(upnsSharedPointer<Branch> &obj, const upnsString &name)
{
    return 0;
}

StatusCode FileSerializer::removeBranch(const upnsString &name)
{
    return 0;
}

upnsSharedPointer<AbstractEntityDataStreamProvider> FileSerializer::getStreamProvider(const ObjectId &entityId, bool canRead, bool canWrite)
{
    upnsSharedPointer<AbstractEntityDataStreamProvider> ptr(new FileEntityDataStreamProvider(entityId));
    return ptr;
}

MessageType FileSerializer::typeOfObject(const ObjectId &oidOrName)
{
    return MessageEmpty;
}

bool FileSerializer::exists(const ObjectId &oidOrName)
{
    return true;
}

StatusCode FileSerializer::cleanUp()
{
    return 0;
}

}
