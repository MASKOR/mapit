#include "operatorloader.h"
#include <upns/serialization/operatorlibrarymanager.h>
void OperatorLoader::run()
{
    QList<QVariant> result;
    std::vector<upns::OperatorInfo> moduleInfos = upns::OperatorLibraryManager::listOperators();
    for(auto iter = moduleInfos.cbegin(); iter != moduleInfos.cend() ; ++iter)
    {
        bool any = false;
        for(auto iter2=result.cbegin() ; iter2 != result.cend() ; ++iter2)
        {
            if(iter2->toJsonObject()["moduleName"].toString() == QString::fromStdString(iter->moduleName))
            {
                any = true;
                break;
            }
        }
        if(any) continue;

        QJsonObject obj;
        obj["compiler"] = QJsonValue(iter->compiler.c_str());
        obj["compilerConfig"] = QJsonValue(iter->compilerConfig.c_str());
        obj["date"] = QJsonValue(iter->date.c_str());
        obj["time"] = QJsonValue(iter->time.c_str());
        obj["moduleName"] = QJsonValue(iter->moduleName.c_str());
        obj["description"] = QJsonValue(iter->description.c_str());
        obj["author"] = QJsonValue(iter->author.c_str());
        obj["moduleVersion"] = iter->moduleVersion;
        obj["apiVersion"] = iter->apiVersion;
        obj["layerType"] = QJsonValue(iter->layerType.c_str());
        result.append(obj);
    }
    operatorsAdded(result);
}

OperatorLoader::OperatorLoader(QObject *parent) : QThread(parent) {}

OperatorLoader::~OperatorLoader() {wait();}
