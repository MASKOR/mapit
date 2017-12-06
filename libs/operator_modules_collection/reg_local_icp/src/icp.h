#ifndef ICP_H
#define ICP_H

#include <list>
#include <string>

class QJsonDocument;

namespace mapit {

class ICP
{
public:
    ICP(QJsonDocument paramsDoc);
private:
    std::list<std::string> cfg_input_;
    std::string cfg_target_;
    bool cfg_use_frame_id_;
    std::string cfg_frame_id_;
    enum class HandleResult {tf_add, tf_change, data_change};
    HandleResult cfg_handle_result_;
    std::string cfg_tf_frame_id_;
    std::string cfg_tf_child_frame_id_;
    std::string cfg_store_prefix_;
};

}
#endif // ICP_H
