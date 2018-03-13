/*******************************************************************************
 *
 * Copyright      2016 Daniel Bulla	<d.bulla@fh-aachen.de>
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
