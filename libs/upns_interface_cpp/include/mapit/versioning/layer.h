#ifndef LAYER_H
#define LAYER_H

#include <list>

#include <mapit/msgs/datastructs.pb.h>
#include <mapit/versioning/entity.h>

namespace mapit
{
class Layer
{
public:
    Layer(std::shared_ptr<mapit::msgs::Tree> layer_tree, std::string name) : layer_(layer_tree), name_(name) { }

    inline std::string getName() { return name_; }

    inline ::google::protobuf::Map< ::std::string, ::mapit::msgs::ObjectReference > getRefs() { return layer_->refs(); }

protected:
    std::shared_ptr<mapit::msgs::Tree> layer_;
    std::string name_;
};
}

#endif // LAYER_H
