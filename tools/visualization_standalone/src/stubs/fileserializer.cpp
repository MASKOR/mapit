//#include "fileserializer.h"
//#include <iostream>
//#include <fstream>
//#include <pcl/PCLPointCloud2.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <upns/layertypes/pointcloud2/src/pointcloudhelper.h"
//#include <boost/archive/text_oarchive.hpp>
//#include <boost/archive/text_iarchive.hpp>
//#include <boost/serialization/vector.hpp>

//namespace upns
//{
//struct membuf: std::streambuf {
//    membuf(char* base, std::ptrdiff_t n) {
//        this->setg(base, base, base + n);
//    }
//};
//class FileEntitydataStreamProvider : public AbstractEntitydataProvider
//{
//    std::string m_filename;
//public:
//    FileEntitydataStreamProvider(const std::string &filename)
//        :m_filename(filename)
//    { }
//    bool isCached() { return true; }
//    bool isReadWriteSame() { return true; }
//    upnsIStream *startRead(upnsuint64 start, upnsuint64 len)
//    {
//        //Note: Usually this is not the place meant to contain pointcloud logic.
//        pcl::PCLPointCloud2 pc2;
//        std::string cachename = m_filename + ".cache";
//        if (FILE *file = fopen(cachename.c_str(), "r")) {
//            fclose(file);
//            return new std::ifstream(cachename);
//        } else {
//            if(m_filename.substr(m_filename.length()-4, std::string::npos) == ".pcd")
//            {
//                pcl::PCDReader reader;
//                if ( reader.read(m_filename, pc2) < 0 )
//                {
//                    log_error("Couldn't read file" + m_filename);
//                    return NULL;
//                }
//            }
//            else
//            {
//                pcl::PLYReader reader;
//                if ( reader.read(m_filename, pc2) < 0 )
//                {
//                    log_error("Couldn't read file" + m_filename);
//                    return NULL;
//                }
//            }
//            std::ofstream cacheout(cachename);
//            std::iostream *is = new std::stringstream();
//            ::boost::archive::text_oarchive oa(*is);
//            oa << pc2;
//            cacheout << is->rdbuf();
//            is->clear();
//            is->seekg(0, std::ios::beg);
//            return is;
//        }
//    }
//    void endRead(upnsIStream *&strm)
//    {
//        //std::ifstream *is = static_cast<std::ifstream*>(strm);
//        //is->close();
//        delete strm;
//    }
//    upnsOStream *startWrite(upnsuint64 start, upnsuint64 len)
//    {
//        std::ofstream *os = new std::ofstream(m_filename.c_str(), std::ifstream::binary);
//        return os;
//    }
//    void endWrite(upnsOStream *&strm)
//    {
//        std::ofstream *os = static_cast<std::ofstream*>(strm);
//        os->close();
//        delete os;
//    }
//    upnsuint64 getStreamSize() const
//    {
//        std::fstream s(m_filename.c_str(), std::fstream::binary);
//        s.seekg (0, s.end);
//        int length = s.tellg();
//        s.seekg (0, s.beg);
//        return length;
//    }
//    void setStreamSize(upnsuint64)
//    {
//        assert(true);
//    }
//    LockHandle lock()
//    {
//        return 0;
//    }
//    void unlock(LockHandle)
//    {
//    }
//};

//bool FileSerializer::canRead()
//{
//    return true;
//}

//bool FileSerializer::canWrite()
//{
//    //TODO: will never work or not yet implemented?
//    return false;
//}

//std::shared_ptr<Tree> FileSerializer::getTree(const ObjectId &oid)
//{
//    return NULL;
//}

//StatusCode FileSerializer::storeTree(std::shared_ptr<Tree> &obj)
//{
//    return 0;
//}

//StatusCode FileSerializer::createTree(std::shared_ptr<Tree> &obj)
//{
//    return 0;
//}

//StatusCode FileSerializer::removeTree(const ObjectId &oid)
//{
//    return 0;
//}

//std::shared_ptr<Entity> FileSerializer::getEntity(const ObjectId oid)
//{
//    std::shared_ptr<Entity> dummyPointcloud2Entity(new Entity());
//    dummyPointcloud2Entity->set_type(LayerType::POINTCLOUD2);
//    return dummyPointcloud2Entity;
//}

//StatusCode FileSerializer::storeEntity(std::shared_ptr<Entity> &obj)
//{
//    return 0;
//}

//StatusCode FileSerializer::createEntity(std::shared_ptr<Entity> &obj)
//{
//    return 0;
//}

//StatusCode FileSerializer::removeEntity(const ObjectId &oid)
//{
//    return 0;
//}

//std::shared_ptr<Commit> FileSerializer::getCommit(const ObjectId &oid)
//{
//    return NULL;
//}

//StatusCode FileSerializer::storeCommit(std::shared_ptr<Commit> &obj)
//{
//    return 0;
//}

//StatusCode FileSerializer::createCommit(std::shared_ptr<Commit> &obj)
//{
//    return 0;
//}

//StatusCode FileSerializer::removeCommit(const ObjectId &oid)
//{
//    return 0;
//}

//std::vector<std::string> FileSerializer::listCheckoutNames()
//{
//    return std::vector<std::string>();
//}

//std::vector<std::shared_ptr<CheckoutObj> > FileSerializer::listCheckouts()
//{
//    return std::vector<std::shared_ptr<CheckoutObj> >();
//}

//std::shared_ptr<CheckoutObj> FileSerializer::getCheckoutCommit(const std::string &name)
//{
//    return NULL;
//}

//StatusCode FileSerializer::storeCheckoutCommit(std::shared_ptr<CheckoutObj> &obj, const std::string &name)
//{
//    return 0;
//}

//StatusCode FileSerializer::createCheckoutCommit(std::shared_ptr<CheckoutObj> &obj, const std::string &name)
//{
//    return 0;
//}

//StatusCode FileSerializer::removeCheckoutCommit(const std::string &name)
//{
//    return 0;
//}

//std::vector<std::shared_ptr<Branch> > FileSerializer::listBranches()
//{
//    return std::vector<std::shared_ptr<Branch> >();
//}

//std::shared_ptr<Branch> FileSerializer::getBranch(const std::string &name)
//{
//    return NULL;
//}

//StatusCode FileSerializer::storeBranch(std::shared_ptr<Branch> &obj, const std::string &name)
//{
//    return 0;
//}

//StatusCode FileSerializer::createBranch(std::shared_ptr<Branch> &obj, const std::string &name)
//{
//    return 0;
//}

//StatusCode FileSerializer::removeBranch(const std::string &name)
//{
//    return 0;
//}

//std::shared_ptr<AbstractEntitydataProvider> FileSerializer::getStreamProvider(const ObjectId &entityId, bool canRead, bool canWrite)
//{
//    std::shared_ptr<AbstractEntitydataProvider> ptr(new FileEntitydataStreamProvider(entityId));
//    return ptr;
//}

//MessageType FileSerializer::typeOfObject(const ObjectId &oidOrName)
//{
//    return MessageEmpty;
//}

//bool FileSerializer::exists(const ObjectId &oidOrName)
//{
//    return true;
//}

//StatusCode FileSerializer::cleanUp()
//{
//    return 0;
//}

//}
