#include <iostream>
#include "leveldb/db.h"
#include "upns.h"
#include "services.pb.h"
#include "versioning/repository.h"
#include <QFile>

int main(int argc, char *argv[])
{
    if(argc != 3)
    {
        std::cout << "usage:\n " << argv[0] << " <config file> <checkout name> <destination>" << std::endl;
        return 1;
    }


    upns::Repository repo = new upns::Repository(std::string(argv[0]));

    upns::upnsSharedPointer<upns::Checkout> co = repo.getCheckout(argv[1]);

    upnsSharedPointer<upns::Tree> currentDirectory(co->getRoot());
    std::string currentPath(argv[2]);

}
