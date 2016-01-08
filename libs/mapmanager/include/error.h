#ifndef UPNS_ERROR_H
#define UPNS_ERROR_H

#define UPNS_STATUS_OK 0

#define upnsIsOk(status) (status == UPNS_STATUS_OK)

#define UPNS_STATUS_ERR_DB_NOT_FOUND 1
#define UPNS_STATUS_ERR_DB_CORRUPTION 2
#define UPNS_STATUS_ERR_DB_NOT_SUPPORTED 3
#define UPNS_STATUS_ERR_DB_INVALID_ARGUMENT 4
#define UPNS_STATUS_ERR_DB_IO_ERROR 5
#define UPNS_STATUS_ERR_DB_UNKNOWN 6

#define UPNS_STATUS_ERR_DB_OPTIMISTIC_LOCKING 7

#define UPNS_STATUS_ERR_DB_PARSE_MAP 8
#define UPNS_STATUS_ERR_DB_DELETE_LAYER_FROM_MAP 9

#define UPNS_STATUS_ERR_MODULE_OPERATOR_NOT_FOUND 10

#define UPNS_STATUS_FILE_NOT_FOUND 11
#define UPNS_STATUS_INVALID_ARGUMENT 12

#define UPNS_STATUS_MAP_NOT_FOUND 13
#define UPNS_STATUS_LAYER_NOT_FOUND 14
#define UPNS_STATUS_ENTITY_NOT_FOUND 15

#define UPNS_STATUS_LAYER_TYPE_MISMATCH 16

#define UPNS_STATUS_ERR_UNKNOWN 666

namespace upns
{

template<typename T>
bool upnsCheckResultVector( T result )
{
    return std::all_of(result.begin(), result.end(), [](typename T::value_type t){return upnsIsOk(t.second);});
}

}

#endif
