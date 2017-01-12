#include "versioning/repositoryfactory.h"
#include "serialization/abstractmapserializer.h"
#include "serialization/file_system/fs_serializer.h"
#include "repositoryimpl.h"

static upns::AbstractMapSerializer *initializeSerializer(std::string directory)
{
    upns::AbstractMapSerializer *mser = new upns::FSSerializer(directory);

    // Check if anything exists in the database
    // Note: There might be commits or objects which are not recognized here.
    // TODO: forbid to delete last branch for this to work. Checkouts might all be deleted.
    size_t numElems = mser->listBranches().size();

    if(numElems) return mser;
//        numElems = m_serializer->listCheckoutNames().size();
//        if(numElems) return;
    log_warn("Selected empty repository, create master");
    upns::upnsSharedPointer<upns::Branch> master(new upns::Branch());
    master->set_commitid(""); //< InitialCommit
    mser->createBranch(master, "master");
    return mser;
}

upns::Repository *upns::RepositoryFactory::openLocalRepository(std::string directory)
{
    upns::upnsSharedPointer<AbstractMapSerializer> mser( initializeSerializer( directory ) );
    return new upns::RepositoryImpl( mser );
}
