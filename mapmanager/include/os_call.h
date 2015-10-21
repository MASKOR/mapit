//Boby Thomas pazheparampil - march 2006
#ifndef os_call_h
#define os_call_h
#include<string>
#if defined(_MSC_VER) // Microsoft compiler
    #include <windows.h>
#elif defined(__GNUC__) // GNU compiler
    #include <dlfcn.h>
#else
#error define your copiler
#endif

/*
#define RTLD_LAZY   1
#define RTLD_NOW    2
#define RTLD_GLOBAL 4
*/

void* LoadSharedLibrary(char *pcDllname, int iMode = 2)
{
    std::string sDllName = pcDllname;
    #if defined(_MSC_VER) // Microsoft compiler
    sDllName += ".dll";
        return (void*)LoadLibrary(pcDllname);
    #elif defined(__GNUC__) // GNU compiler
    sDllName += ".so";
        return dlopen(sDllName.c_str(),iMode);
    #endif
}
void *GetFunction(void *Lib, char *Fnname)
{
#if defined(_MSC_VER) // Microsoft compiler
    return (void*)GetProcAddress((HINSTANCE)Lib,Fnname);
#elif defined(__GNUC__) // GNU compiler
    return dlsym(Lib,Fnname);
#endif
}

bool FreeSharedLibrary(void *hDLL)
{
#if defined(_MSC_VER) // Microsoft compiler
    return FreeLibrary((HINSTANCE)hDLL);
#elif defined(__GNUC__) // GNU compiler
    return dlclose(hDLL);
#endif
}


#endif //os_call_h