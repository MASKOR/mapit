#include "util.h"
//#include <random>

//class static_init
//{
//public:
//    static_init()
//    {
//        srand (0);//time(NULL));
//    }
//};

//static static_init _static_init;

bool upns::protobufContains(::google::protobuf::RepeatedPtrField< ::std::string> *field, const ::std::string &str)
{
    ::google::protobuf::RepeatedPtrField< ::std::string>::const_iterator iter(field->cbegin());
    while(iter != field->cend())
    {
        if(*iter == str) return true;
    }
    return false;
}
