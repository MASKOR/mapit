#ifndef ENTITY_H
#define ENTITY_H

#include <list>

#include <mapit/time/time.h>
#include <mapit/msgs/datastructs.pb.h>

namespace mapit
{
class Entity
{
public:
    Entity(std::shared_ptr<mapit::msgs::Entity> entity_tree, std::string name, std::string entity_data_path) : entity_(entity_tree), name_(name), entity_data_path_(entity_data_path){ }

    inline std::string getName() { return name_; }
    inline std::string getEntityDataPath() { return entity_data_path_; }

    inline std::shared_ptr<mapit::msgs::Entity> getEntity() { return entity_; }
    inline std::string frame_id() { return entity_->frame_id(); }
    inline mapit::time::Stamp stamp() { return mapit::time::from_sec_and_nsec( entity_->stamp().sec(), entity_->stamp().nsec() ); }

protected:
    std::shared_ptr<mapit::msgs::Entity> entity_;
    std::string name_;
    std::string entity_data_path_;
};
}

#endif // ENTITY_H
