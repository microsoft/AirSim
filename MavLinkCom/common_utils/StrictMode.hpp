#ifndef common_StrictMode_hpp
#define common_StrictMode_hpp

#if defined(_MSC_VER)
//'=': conversion from 'double' to 'float', possible loss of data
#define STRICT_MODE_OFF                                           \
    __pragma(warning(push))										  \
    __pragma(warning( disable : 4100 4189 4244 4245 4239 4464 4456 4505 4514 4571 4624 4626 4267 4710 4820 5027 5031))					  
#define STRICT_MODE_ON                                            \
    __pragma(warning(pop))										  

//TODO: limit scope of below statements required to suppress VC++ warnings
#define _CRT_SECURE_NO_WARNINGS 1
#pragma warning(disable:4996)
#endif


#ifdef __GNUC__
#define STRICT_MODE_OFF                                           \
    _Pragma("GCC diagnostic push")                                  \
    _Pragma("GCC diagnostic ignored \"-Wctor-dtor-privacy\"")        \
    _Pragma("GCC diagnostic ignored \"-Wdelete-non-virtual-dtor\"") \
    _Pragma("GCC diagnostic ignored \"-Wmissing-field-initializers\"") \
    _Pragma("GCC diagnostic ignored \"-Wold-style-cast\"")          \
    _Pragma("GCC diagnostic ignored \"-Wredundant-decls\"")         \
    _Pragma("GCC diagnostic ignored \"-Wreturn-type\"")             \
    _Pragma("GCC diagnostic ignored \"-Wshadow\"")                  \
    _Pragma("GCC diagnostic ignored \"-Wswitch-default\"")          \
    _Pragma("GCC diagnostic ignored \"-Wundef\"")                   \
    _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"")

/* Addition options that can be enabled 
    _Pragma("GCC diagnostic ignored \"-Wpedantic\"")                \
    _Pragma("GCC diagnostic ignored \"-Wformat=\"")                 \
    _Pragma("GCC diagnostic ignored \"-Werror\"")                   \
    _Pragma("GCC diagnostic ignored \"-Werror=\"")                  \
    _Pragma("GCC diagnostic ignored \"-Wunused-variable\"")         \
*/

#define STRICT_MODE_ON                                            \
    _Pragma("GCC diagnostic pop")          
#endif


#endif