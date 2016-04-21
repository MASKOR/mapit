#include <iostream>
#include "leveldb/db.h"
#include "upns.h"
#include "services.pb.h"
#include "versioning/repository.h"
#include <QFile>
#include <QDir>
#include <yaml-cpp/yaml.h>

int main(int argc, char *argv[])
{
    if(argc != 3)
    {
        std::cout << "usage:\n " << argv[0] << " <config file> <checkout name> <destination>" << std::endl;
        return 1;
    }
    YAML::Node config = YAML::LoadFile(std::string(argv[0]));

    upns::Repository repo( config );

    upns::upnsSharedPointer<upns::Checkout> co = repo.getCheckout(argv[1]);

    upns::upnsSharedPointer<upns::Tree> currentDirectory(co->getRoot());
    std::string rootPath(argv[2]);
    if(rootPath.at(rootPath.length()-1) != '/')
    {
        rootPath = rootPath.substr(0, rootPath.length()-1);
    }
    std::string::size_type idx = rootPath.find_last_of('/');
    std::string folderName = rootPath.substr(idx, rootPath.length()-idx);
    if(folderName != argv[1])
    {
        rootPath += argv[1];
    }
    upns::StatusCode s = co->depthFirstSearch([&](
        upns::upnsSharedPointer<upns::Commit> obj, const upns::ObjectId& oid, const upns::Path &path)
        {
            return true;
        },
        [&](upns::upnsSharedPointer<upns::Commit> obj, const upns::ObjectId& oid, const upns::Path &path)
        {
            return true;
        },
        [&](upns::upnsSharedPointer<upns::Tree> obj, const upns::ObjectId& oid, const upns::Path &path)
        {
            QDir current(QString::fromStdString(rootPath));
            if(!current.exists())
            {
                current.mkpath(QString::fromStdString(path));
            }
            return true;
        }, [&](upns::upnsSharedPointer<upns::Tree> obj, const upns::ObjectId& oid, const upns::Path &path)
        {
            return true;
        },
        [&](upns::upnsSharedPointer<upns::Entity> obj, const upns::ObjectId& oid, const upns::Path &path)
        {
            QFile current(QString::fromStdString(rootPath + path));
            if(current.exists())
            {
                 //QByteArray ar = current.readAll();
                 //Hash
            }
            upns::upnsSharedPointer<upns::AbstractEntityData> reader = co->getEntityDataReadOnly(oid);
            upns::upnsIStream *istrm = reader->startReadBytes();
            char buffer[4096];
            while (istrm->read(buffer, sizeof(buffer)))
                current.write(buffer, 4096);
            current.write(buffer, istrm->gcount());
            reader->endRead(istrm);
            return true;
        },
        [&](upns::upnsSharedPointer<upns::Entity> obj, const upns::ObjectId& oid, const upns::Path &path)
        {
            return true;
        });

}
