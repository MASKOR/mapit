//#ifndef FILESERIALIZER_H
//#define FILESERIALIZER_H

//#include "serialization/abstractmapserializer.h"
//namespace upns
//{

//class FileSerializer : public upns::AbstractSerializer
//{


//    // AbstractSerializer interface
//public:
//    bool canRead();
//    bool canWrite();
//    std::shared_ptr<Tree> getTree(const ObjectId &oid);
//    std::pair<StatusCode, ObjectId> storeTree(std::shared_ptr<Tree> &obj);
//    StatusCode createTree(std::shared_ptr<Tree> &obj);
//    StatusCode removeTree(const ObjectId &oid);
//    std::shared_ptr<Entity> getEntity(const ObjectId oid);
//    std::pair<StatusCode, ObjectId> storeEntity(std::shared_ptr<Entity> &obj);
//    StatusCode createEntity(std::shared_ptr<Entity> &obj);
//    StatusCode removeEntity(const ObjectId &oid);
//    std::shared_ptr<Commit> getCommit(const ObjectId &oid);
//    StatusCode storeCommit(std::shared_ptr<Commit> &obj);
//    std::pair<StatusCode, ObjectId> createCommit(std::shared_ptr<Commit> &obj);
//    StatusCode removeCommit(const ObjectId &oid);
//    std::vector<std::string> listCheckoutNames();
//    std::vector<std::shared_ptr<CheckoutObj> > listCheckouts();
//    std::shared_ptr<CheckoutObj> getCheckoutCommit(const std::string &name);
//    StatusCode storeCheckoutCommit(std::shared_ptr<CheckoutObj> &obj, const std::string &name);
//    StatusCode createCheckoutCommit(std::shared_ptr<CheckoutObj> &obj, const std::string &name);
//    StatusCode removeCheckoutCommit(const std::string &name);
//    std::vector<std::shared_ptr<Branch> > listBranches();
//    std::shared_ptr<Branch> getBranch(const std::string &name);
//    StatusCode storeBranch(std::shared_ptr<Branch> &obj, const std::string &name);
//    StatusCode createBranch(std::shared_ptr<Branch> &obj, const std::string &name);
//    StatusCode removeBranch(const std::string &name);
//    std::shared_ptr<AbstractEntitydataProvider> getStreamProvider(const ObjectId &entityId, bool canRead, bool canWrite);
//    MessageType typeOfObject(const ObjectId &oidOrName);
//    bool exists(const ObjectId &oidOrName);
//    StatusCode cleanUp();
//};

//}
//#endif
