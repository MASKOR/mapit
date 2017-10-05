#ifndef ENTITY_H
#define ENTITY_H

#include <list>

#include <mapit/time/time.h>
#include <mapit/msgs/datastructs.pb.h>
#include <mapit/versioning/layer.h>

namespace mapit
{
class Entity
{
public:
    Entity(std::shared_ptr<mapit::msgs::Entity> entity_tree, std::string name, std::shared_ptr<mapit::Layer> layer)
        : entity_(entity_tree),
          name_(name),
          layer_(layer)
    { }

    inline const std::string& getName() { return name_; }
    inline std::string getDataPath()
    {
        return layer_->getDataPath() + getName() + "/";
    }

    inline std::shared_ptr<mapit::msgs::Entity> getEntity() { return entity_; }
    inline const std::string& frame_id() { return entity_->frame_id(); }
    inline mapit::time::Stamp stamp() { return mapit::time::from_sec_and_nsec( entity_->stamp().sec(), entity_->stamp().nsec() ); }

    inline void set_frame_id(const std::string& frame_id)
    {
        entity_->set_frame_id(frame_id);
    }

    inline void set_stamp(const mapit::time::Stamp& stamp)
    {
        unsigned long sec, nsec;
        mapit::time::to_sec_and_nsec(stamp, sec, nsec);

        mapit::msgs::Time* stamp_mutable = entity_->mutable_stamp();
        stamp_mutable->set_sec(sec);
        stamp_mutable->set_nsec(nsec);
    }

protected:
    std::shared_ptr<mapit::msgs::Entity> entity_;
    std::string name_;
    std::shared_ptr<mapit::Layer> layer_;
};
}

#endif // ENTITY_H
