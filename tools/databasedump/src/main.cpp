#include <iostream>
#include "leveldb/db.h"

int main(int argc, char *argv[])
{
    if(argc != 2 && argc != 3)
    {
        std::cout << "usage:\n " << argv[0] << " <database file> [key]" << std::endl;
        return 1;
    }
    leveldb::Options options;
    leveldb::DB *db;
    options.create_if_missing = false;
    leveldb::Status status = leveldb::DB::Open(options, argv[1], &db);
    assert(status.ok());

    if(argc == 2)
    {
        leveldb::Iterator* it = db->NewIterator(leveldb::ReadOptions());
        it->SeekToFirst();
        while(it->Valid())
        {
            std::cout << "KEY: ";
            std::string key(it->key().data(), it->key().size());
            std::string value(it->value().data(), it->value().size());
            std::cout.write(key.c_str(), key.length());
            std::cout << ": ";
            if(value.length()>100) std::cout << "\n[BIG VALUE START]:\n";
            std::cout.write(value.c_str(), value.length()<300?value.length():300);
            if(!value.length()<300) std::cout << "(...)";
            if(value.length()>100) std::cout << "\n[BIG VALUE END]";
            std::cout << ", Size: " << it->value().size() << std::endl;
            it->Next();
        }
        delete it;
    }
    else if(argc == 3)
    {
        std::string val;
        db->Get(leveldb::ReadOptions(), argv[2], &val);
        std::cout.write(val.data(), val.length());
//        leveldb::Iterator* it = m_db->NewIterator(leveldb::ReadOptions());
//        it->Seek(m_readkey);
    }
}
