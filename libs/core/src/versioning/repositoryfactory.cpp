#include "upns/versioning/repositoryfactory.h"
#include "serialization/abstractserializer.h"
#include "serialization/file_system/fs_serializer.h"
#include "repositoryimpl.h"

static upns::AbstractSerializer *initializeSerializer(std::string directory)
{
    upns::AbstractSerializer *mser = new upns::FSSerializer(directory);

    // Check if anything exists in the database
    // Note: There might be commits or objects which are not recognized here.
    // TODO: forbid to delete last branch for this to work. Checkouts might all be deleted.
    size_t numElems = mser->listBranches().size();

    if(numElems) return mser;
//        numElems = m_serializer->listCheckoutNames().size();
//        if(numElems) return;
    log_warn("Selected empty repository, create master");
    std::shared_ptr<upns::Branch> master(new upns::Branch());
    master->set_commitid(""); //< InitialCommit
    mser->createBranch(master, "master");
    return mser;
}

upns::Repository *upns::RepositoryFactory::openLocalRepository(std::string directory)
{
    std::shared_ptr<AbstractSerializer> mser( initializeSerializer( directory ) );
    return new upns::RepositoryImpl( mser );
}
