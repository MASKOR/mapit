#include <iostream>
#include "leveldb/db.h"
#include "upns.h"
#include "services.pb.h"
#include "versioning/repository.h"
#include <QFile>
#include <QDir>
#include <yaml-cpp/yaml.h>
#include <log4cplus/configurator.h>
#include <log4cplus/consoleappender.h>

int main(int argc, char *argv[])
{
    log4cplus::BasicConfigurator logconfig;
    logconfig.configure();
    log4cplus::SharedAppenderPtr consoleAppender(new log4cplus::ConsoleAppender());
    consoleAppender->setName("myAppenderName");
    consoleAppender->setLayout(std::auto_ptr<log4cplus::Layout>(new log4cplus::TTCCLayout()));
    log4cplus::Logger mainLogger = log4cplus::Logger::getInstance("main");
    mainLogger.addAppender(consoleAppender);
    if(argc != 4)
    {
        std::cout << "usage:\n " << argv[0] << " <config file> <checkout name> <destination>" << std::endl;
        return 1;
    }
    YAML::Node config = YAML::LoadFile(std::string(argv[1]));

    upns::Repository repo( config );

    upns::upnsSharedPointer<upns::Checkout> co = repo.getCheckout(argv[2]);

    upns::upnsSharedPointer<upns::Tree> currentDirectory(co->getRoot());
    std::string rootPath(argv[3]);
    if(rootPath.at(rootPath.length()-1) != '/')
    {
        // ensure trailing "/"
        rootPath += "/";
    }
    std::string::size_type idx = rootPath.find_last_of('/', rootPath.length()-2);
    std::string folderName;
    // get last segment of path
    if(idx == std::string::npos)
    {
        folderName = rootPath;
    }
    else
    {
        folderName = rootPath.substr(idx, rootPath.length()-idx-1);
    }
    // if last segment is equal to checkoutname, use it.
    if(folderName != argv[2])
    {
        // otherwise, use subfolder with checkout name
        rootPath += argv[2];
    }
    if(rootPath.at(rootPath.length()-1) == '/')
    {
        // ensure trailing "/" again
        rootPath = rootPath.substr(0, rootPath.length()-1);
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
            if(!path.empty() && !current.exists(QString::fromStdString(path)))
            {
                if(!current.mkpath(QString::fromStdString(path.substr(1, path.length()-1))))
                {
                    log_error("Path " + path + " could not be created");
                    return false;
                }
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
            else
            {
            }
            if(current.open(QFile::WriteOnly))
            {
                upns::upnsSharedPointer<upns::AbstractEntityData> reader = co->getEntityDataReadOnly(path);
                upns::upnsIStream *istrm = reader->startReadBytes();
                char buffer[4096];
                while (istrm->read(buffer, sizeof(buffer)))
                    current.write(buffer, 4096);
                current.write(buffer, istrm->gcount());
                reader->endRead(istrm);
                current.close();
            }
            else
            {
                log_error("could not write entity: " + path);
            }
            return true;
        },
        [&](upns::upnsSharedPointer<upns::Entity> obj, const upns::ObjectId& oid, const upns::Path &path)
        {
            return true;
        });

}
