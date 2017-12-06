#ifndef ICP_H
#define ICP_H

#include <memory>
#include <list>
#include <string>
#include <upns/typedefs.h>

class QJsonDocument;
namespace upns {
class OperationEnvironment;
class CheckoutRaw;
}

namespace mapit {
namespace tf2 {
class BufferCore;
}

class ICP
{
public:
    ICP(upns::OperationEnvironment* env, upns::StatusCode& status);

    upns::StatusCode operate();
private:
    upns::CheckoutRaw* checkout_;
    std::shared_ptr<mapit::tf2::BufferCore> tf_buffer_;

    std::list<std::string> cfg_input_;
    std::string cfg_target_;
    bool cfg_use_frame_id_;
    std::string cfg_frame_id_;
    enum class HandleResult {tf_add, tf_change, data_change};
    HandleResult cfg_handle_result_;
    std::string cfg_tf_frame_id_;
    std::string cfg_tf_child_frame_id_;
    std::string cfg_tf_prefix_;
};

}
#endif // ICP_H
