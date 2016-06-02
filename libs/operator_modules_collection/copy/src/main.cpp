#include "module.h"
#include "modules/versioning/checkoutraw.h"
#include "operationenvironment.h"
#include "error.h"
#include "modules/versioning/checkoutraw.h"
#include <QJsonDocument>
#include <QJsonObject>

upns::StatusCode operate(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string source = params["source"].toString().toStdString();
    std::string target = params["target"].toString().toStdString();

    if(source.empty())
    {
        log_error("could not copy, source is not set");
    }
    if(target.empty())
    {
        log_error("could not copy, target is not set");
    }

    upns::upnsSharedPointer< upns::Entity > srcEnt( env->getCheckout()->getEntity(source) );
    upns::StatusCode s;
    if(srcEnt)
    {
        s = env->getCheckout()->storeEntity(target, srcEnt);
    }
    else
    {
        upns::upnsSharedPointer< upns::Tree > srcTree( env->getCheckout()->getTree(source) );
        if(srcTree)
        {
            s = env->getCheckout()->storeTree(target, srcTree);
        }
        else
        {
            s = UPNS_STATUS_ERR_DB_NOT_FOUND;
            log_error("Path not found " + source);
        }
    }
    if(!upnsIsOk(s))
    {
        log_error("Could not copy \"" + source + "\" to \"" + target + "\"");
    }
    else
    {
        log_info("copied \"" + source + "\" to \"" + target + "\"");
    }

    upns::OperationDescription out;
    out.set_operatorname(OPERATOR_NAME);
    out.set_operatorversion(OPERATOR_VERSION);
//    OperationParameter *outTarget = out.add_params();
//    outTarget->set_key("target");
////    outTarget->set_mapval( map->id() );
////    outTarget->set_layerval( layer->id() );
////    outTarget->set_entityval( entity->id() );
//    OperationParameter *outMapname = out.add_params();
//    outMapname->set_key("leafsize");
//    outMapname->set_realval( leafSize );
    env->setOutputDescription( out.SerializeAsString() );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "copy tree or entity", "fhac", OPERATOR_VERSION, upns::LayerType::POINTCLOUD2, &operate)
