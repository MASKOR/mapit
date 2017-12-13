#ifndef LAYER_H
#define LAYER_H

#include <list>

#include <mapit/msgs/datastructs.pb.h>
#include <mapit/versioning/map.h>

namespace mapit
{
class Layer
{
public:
    Layer(std::shared_ptr<mapit::msgs::Tree> layer_tree, std::string name, std::shared_ptr<mapit::Map> map, std::string type)
        : layer_(layer_tree)
        , name_(name)
        , map_(map)
        , type_(type)
    { }

    inline std::string getName() { return name_; }
    inline std::string getDataPath()
    {
        return map_->getDataPath() + getName() + "/";
    }
    inline const std::string& getTypeString()
    {
      return type_;
    }

    inline std::shared_ptr<mapit::Map> getMap() { return map_; }

    inline const ::google::protobuf::Map< ::std::string, ::mapit::msgs::ObjectReference >& getRefs() { return layer_->refs(); }

protected:
    std::shared_ptr<mapit::msgs::Tree> layer_;
    std::string name_;
    std::shared_ptr<mapit::Map> map_;
    std::string type_;
};
}

#endif // LAYER_H
