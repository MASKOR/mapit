#include "versioning/repositoryfactory.h"
#include "serialization/abstractmapserializer.h"
#include "serialization/leveldb/leveldbserializer.h"
#include "serialization/file_system/fs_serializer.h"
#include "repositoryimpl.h"

static upns::AbstractMapSerializer *initializeSerializer(const YAML::Node &config)
{
    upns::AbstractMapSerializer *mser = NULL;
    if(const YAML::Node mapsource = config["mapsource"])
    {
        if(const YAML::Node mapsourceName = mapsource["name"])
        {
            std::string mapsrcnam = mapsourceName.as<std::string>();
            std::transform(mapsrcnam.begin(), mapsrcnam.end(), mapsrcnam.begin(), ::tolower);
            if(mapsrcnam == "leveldb")
            {
                //TODO: Linker Error: SHA multiple definitions (when the following line is uncommented)
                mser = new upns::LevelDBSerializer(mapsource);
            }
            else if(mapsrcnam == "filesystem")
            {
                mser = new upns::FSSerializer(mapsource);
            }
            else
            {
                log_error("mapsource '" + mapsrcnam + "' was not found.");
            }
        } else {
            log_error("'mapsource' has no 'name' in config");
        }
    } else {
        log_error("Key 'mapsource' not given in config");
    }
    assert(mser);
    // Check if anything exists in the database
    // Note: There might be commits or objects which are not recognized here.
    // TODO: forbid to delete last branch for this to work. Checkouts might all be deleted.
    size_t numElems = mser->listBranches().size();
    if(numElems) return mser;
//        numElems = m_serializer->listCheckoutNames().size();
//        if(numElems) return;
    upns::upnsSharedPointer<upns::Branch> master(new upns::Branch());
    master->set_commitid(""); //< InitialCommit
    mser->createBranch(master, "master");
    return mser;
}



upns::Repository *upns::RepositoryFactory::openLocalRepository(const upns::upnsString &filename)
{
    YAML::Node config = YAML::LoadFile( filename );
    upns::upnsSharedPointer<AbstractMapSerializer> mser( initializeSerializer( config ) );
    return new upns::RepositoryImpl( mser );
}

upns::Repository *upns::RepositoryFactory::openLocalRepository(const YAML::Node &config)
{
    upns::upnsSharedPointer<AbstractMapSerializer> mser( initializeSerializer( config ) );
    return new upns::RepositoryImpl( mser );
}
