#include <upns/errorcodes.h>
#include <upns/logging.h>
#include <upns/operators/module.h>
#include <upns/operators/operationenvironment.h>
#include <QJsonDocument>
#include <QJsonObject>
#include <cmath>

upns::StatusCode operateMovingLeastSquares(upns::OperationEnvironment* environment)
{
    log_info("┌surface: moving least squares");
    QByteArray parametersRaw(environment->getParameters().c_str(),
                             environment->getParameters().length());
    QJsonObject parameters(QJsonDocument::fromJson(parametersRaw).object());
    log_info("├─┬parameters");

    std::string source = parameters["source"].toString().toStdString();
    if (source.empty()) {
        log_error("└─┴─source entity string is empty");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    log_info("│ ├─source: '" << source << "'");

    std::string target = parameters["target"].toString().toStdString();
    if (source.empty()) {
        log_error("└─┴─target entity string is empty");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    log_info("│ ├─target: '" << target << "'");

    std::double_t searchRadius = parameters["searchRadius"].toDouble();
    if (!std::isfinite(searchRadius) || searchRadius <= 0.0) {
        log_error("└─┴─search radius is smaller than or equal to zero");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    log_info("│ ├─search radius: " << searchRadius);

    std::double_t variance = parameters["variance"].toDouble();
    if (!std::isfinite(variance) || variance <= 0.0) {
        log_error("└─┴─variance is smaller than or equal to zero");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    log_info("│ ├─variance: " << variance);

    bool polynomialFit= parameters["polynomialFit"].toBool();
    log_info("│ ├─polynomial fit: " << (polynomialFit ? "true" : "false"));

    std::int32_t polynomialOrder = -1;
    if (polynomialFit == true) {
        polynomialOrder = parameters["polynomialOrder"].toDouble();
        if (!polynomialOrder <= 0) {
            log_error("└─┴─polynomial order is smaller than or equal to zero");
            return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
        }
        log_info("│ ├─polynomial order: " << polynomialOrder);
    }

    bool calculateNormals= parameters["calculateNormals"].toBool();
    log_info("│ └─calculate normals: " << (calculateNormals ? "true" : "false"));

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME,
            "surface: moving least squares",
            "Marcus Meeßen",
            OPERATOR_VERSION,
            PointcloudEntitydata_TYPENAME,
            &operateMovingLeastSquares)
