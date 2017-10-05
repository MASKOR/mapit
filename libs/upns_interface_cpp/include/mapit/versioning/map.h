#ifndef MAP_H
#define MAP_H

#include <list>

#include <mapit/msgs/datastructs.pb.h>
#include <mapit/versioning/layer.h>

namespace mapit
{
class Map
{
public:
    Map(std::shared_ptr<mapit::msgs::Tree> map_tree, std::string name) : map_(map_tree), name_(name) { }

    inline std::string getName() { return name_; }

    inline ::google::protobuf::Map< ::std::string, ::mapit::msgs::ObjectReference > getRefs() { return map_->refs(); }

protected:
    std::shared_ptr<mapit::msgs::Tree> map_;
    std::string name_;
};
}

#endif // MAP_H
