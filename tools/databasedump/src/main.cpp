#include <iostream>
#include "leveldb/db.h"

int main(int argc, char *argv[])
{
    if(argc != 2)
    {
        std::cout << "usage:\n " << argv[0] << " <database file>" << std::endl;
        return 1;
    }
    leveldb::Options options;
    leveldb::DB *db;
    options.create_if_missing = true;
    leveldb::Status status = leveldb::DB::Open(options, argv[1], &db);
    assert(status.ok());

    leveldb::Iterator* it = db->NewIterator(leveldb::ReadOptions());
    it->SeekToFirst();
    while(it->Valid())
    {
        std::string key(it->key().data());
        std::string value(it->value().data());
        std::cout << key.c_str() << " : " << value.c_str() << std::endl;
        it->Next();
    }
    delete it;
}
