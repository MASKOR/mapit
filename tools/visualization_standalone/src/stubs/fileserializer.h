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
//    upnsSharedPointer<Tree> getTree(const ObjectId &oid);
//    upnsPair<StatusCode, ObjectId> storeTree(upnsSharedPointer<Tree> &obj);
//    StatusCode createTree(upnsSharedPointer<Tree> &obj);
//    StatusCode removeTree(const ObjectId &oid);
//    upnsSharedPointer<Entity> getEntity(const ObjectId oid);
//    upnsPair<StatusCode, ObjectId> storeEntity(upnsSharedPointer<Entity> &obj);
//    StatusCode createEntity(upnsSharedPointer<Entity> &obj);
//    StatusCode removeEntity(const ObjectId &oid);
//    upnsSharedPointer<Commit> getCommit(const ObjectId &oid);
//    StatusCode storeCommit(upnsSharedPointer<Commit> &obj);
//    upnsPair<StatusCode, ObjectId> createCommit(upnsSharedPointer<Commit> &obj);
//    StatusCode removeCommit(const ObjectId &oid);
//    upnsVec<upnsString> listCheckoutNames();
//    upnsVec<upnsSharedPointer<CheckoutObj> > listCheckouts();
//    upnsSharedPointer<CheckoutObj> getCheckoutCommit(const upnsString &name);
//    StatusCode storeCheckoutCommit(upnsSharedPointer<CheckoutObj> &obj, const upnsString &name);
//    StatusCode createCheckoutCommit(upnsSharedPointer<CheckoutObj> &obj, const upnsString &name);
//    StatusCode removeCheckoutCommit(const upnsString &name);
//    upnsVec<upnsSharedPointer<Branch> > listBranches();
//    upnsSharedPointer<Branch> getBranch(const upnsString &name);
//    StatusCode storeBranch(upnsSharedPointer<Branch> &obj, const upnsString &name);
//    StatusCode createBranch(upnsSharedPointer<Branch> &obj, const upnsString &name);
//    StatusCode removeBranch(const upnsString &name);
//    upnsSharedPointer<AbstractEntitydataStreamProvider> getStreamProvider(const ObjectId &entityId, bool canRead, bool canWrite);
//    MessageType typeOfObject(const ObjectId &oidOrName);
//    bool exists(const ObjectId &oidOrName);
//    StatusCode cleanUp();
//};

//}
//#endif
