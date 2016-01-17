#ifndef UPNS_HASH_H
#define UPNS_HASH_H

#include "upns_globals.h"
#include "services.pb.h"
#include <google/protobuf/message.h>
#include <string>
#include <sstream>
#include <functional>
//#include <openssl/sha.h>
#include <sha256.h>

namespace upns {

//template<typename T>
//void hash(SHA256_CTX *ctx, const T &t);
//void hash(SHA256_CTX *ctx, const Tree &t);
//void hash(SHA256_CTX *ctx, const Branch &t);
//void hash(SHA256_CTX *ctx, const Entity &t);
//void hash(SHA256_CTX *ctx, const TransformRef &t);
//void hash(SHA256_CTX *ctx, const ObjectReference &t);
//void hash(SHA256_CTX *ctx, const Commit &t);
//void hash(SHA256_CTX *ctx, const OperationDescription &t);
//void hash(SHA256_CTX *ctx, const OperationParameter &t);
//void hash(SHA256_CTX *ctx, const Commit::ObjectIdList &t);
//void hash(SHA256_CTX *ctx, const Commit::ObjectIdPair &t);

template<typename T>
ObjectId hash_toString(const T *t)
{
    return sha256(t->SerializeAsString());
//    SHA256_CTX shaCtx;
//    SHA256_Init(&shaCtx);
//    hash(&shaCtx, *t);
//    char szBuff[256];
//    SHA256_Final(reinterpret_cast<unsigned char*>(szBuff), &shaCtx);
//    return std::string(szBuff);
}

////template<>
////ObjectId hash(const ::google::protobuf::Message &t)
////{
////    std::hash<std::string> stdHash;
////    std::stringstream strstr;
////    strstr << stdHash(t.SerializeAsString());
////    return strstr.str();
////}

//void hash(SHA256_CTX *ctx, const Tree &t)
//{
//    ::google::protobuf::Map< ::std::string, ::upns::ObjectReference >::const_iterator iter(t.refs().cbegin());
//    while(iter != t.refs().cend())
//    {
//        SHA256_Update(ctx, iter->first.data(), iter->first.length());
//        hash(ctx, iter->second);
//        iter++;
//    }
//}

//void hash(SHA256_CTX *ctx, const Branch &t)
//{
//    //SHA256_Update(ctx, t.name().data(), t.name().length());
//    SHA256_Update(ctx, t.commitid().data(), t.commitid().length());
//}

//void hash(SHA256_CTX *ctx, const Entity &t)
//{
//    SHA256_Update(ctx, t.dataid().data(), t.dataid().length());
//    const ::google::protobuf::int64 lastChange = t.lastchange();
//    SHA256_Update(ctx, &lastChange, sizeof(lastChange));
//    LayerType type = t.type();
//    SHA256_Update(ctx, &type, sizeof(type));
//    LayerUsageType usageType = t.usagetype();
//    SHA256_Update(ctx, &usageType, sizeof(usageType));
//    hash(ctx, t.transform());
//}

//void hash(SHA256_CTX *ctx, const TransformRef &t)
//{
//    SHA256_Update(ctx, t.parentid().data(), t.parentid().length());
//    const ::google::protobuf::int64 ts = t.timestamp();
//    SHA256_Update(ctx, &ts, sizeof(ts));
//}

//void hash(SHA256_CTX *ctx, const ObjectReference &t)
//{
//    SHA256_Update(ctx, t.meta().data(), t.meta().length());
//}

//void hash(SHA256_CTX *ctx, const Commit &t)
//{
//    ::google::protobuf::RepeatedPtrField< ::std::string >::const_iterator iter(t.parentcommitids().cbegin());
//    while(iter != t.parentcommitids().cend())
//    {
//        SHA256_Update(ctx, iter->data(), iter->length());
//        iter++;
//    }
//    ::google::protobuf::RepeatedPtrField<  OperationDescription >::const_iterator iterOpDesc(t.ops().cbegin());
//    while(iterOpDesc != t.ops().cend())
//    {
//        hash(ctx, *iterOpDesc);
//        iterOpDesc++;
//    }
//    ::google::protobuf::RepeatedPtrField<  Commit::ObjectIdPair >::const_iterator iterTrans(t.transitions().cbegin());
//    while(iterTrans != t.transitions().cend())
//    {
//        hash(ctx, *iterTrans);
//        iterTrans++;
//    }
//    ::google::protobuf::RepeatedPtrField<  Commit::ObjectIdList >::const_iterator iterIdList(t.detailedtransitions().cbegin());
//    while(iterIdList != t.detailedtransitions().cend())
//    {
//        hash(ctx, *iterIdList);
//        iterIdList++;
//    }
//    SHA256_Update(ctx, t.root().data(), t.root().length());
//}

//void hash(SHA256_CTX *ctx, const OperationDescription &t)
//{
//    const ::google::protobuf::int32 ver = t.operatorversion();
//    SHA256_Update(ctx, t.operatorname().data(), t.operatorname().length());
//    SHA256_Update(ctx, &ver, sizeof(::google::protobuf::int32));
//    ::google::protobuf::RepeatedPtrField<  OperationParameter >::const_iterator iter(t.params().cbegin());
//    while(iter != t.params().cend())
//    {
//        hash(ctx, *iter);
//        iter++;
//    }
//}

//void hash(SHA256_CTX *ctx, const OperationParameter &t)
//{
//    const ::google::protobuf::int64 intVal = t.intval();
//    const double dVal = t.realval();
//    SHA256_Update(ctx, t.key().data(), t.key().length());
//    SHA256_Update(ctx, &intVal, sizeof(::google::protobuf::int64));
//    SHA256_Update(ctx, &dVal, sizeof(double));
//    SHA256_Update(ctx, t.strval().data(), t.strval().length());
//    SHA256_Update(ctx, t.objectid().data(), t.objectid().length());
//}

//void hash(SHA256_CTX *ctx, const Commit::ObjectIdPair &t)
//{
//    SHA256_Update(ctx, t.sourceobjectid().data(), t.sourceobjectid().length());
//    SHA256_Update(ctx, t.parentobjectid().data(), t.parentobjectid().length());
//}

//void hash(SHA256_CTX *ctx, const Commit::ObjectIdList &t)
//{
//    SHA256_Update(ctx, t.sourceobjectid().data(), t.sourceobjectid().length());
//    ::google::protobuf::RepeatedPtrField<  ::std::string >::const_iterator iter(t.objectid().cbegin());
//    while(iter != t.objectid().cend())
//    {
//        SHA256_Update(ctx, t.sourceobjectid().data(), t.sourceobjectid().length());
//        iter++;
//    }
//    //Note: Different structures of the list could have the same hash. To avoid this, we hash a delimiter.
//    std::string delim("listEnd");
//    SHA256_Update(ctx, delim.data(), delim.length());
//}

}
#endif
