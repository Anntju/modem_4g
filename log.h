#include <stdio.h>

#if 0
#define  PR(...) printf(__VA_ARGS__) 
#define  RLOGD(...) printf(__VA_ARGS__) 
#define  RLOGI(...) printf(__VA_ARGS__) 
#define  RLOGE(...) printf(__VA_ARGS__) 
#endif

#define LOG_OFF             0 // close all log
#define LOG_ERROR           1
#define LOG_WARN            2
#define LOG_INFO            3
#define LOG_DEBUG           4
 
extern int current_dbg_level;


#define IOT_PRINT_RAW(Level, _fmt, ...)\
{\
    if (Level <= current_dbg_level)\
    {\
        printf(_fmt, ##__VA_ARGS__);\
    }\
}

#define RLOGE(Fmt, ...)           \
{                                   \
    IOT_PRINT_RAW(LOG_ERROR, Fmt, ##__VA_ARGS__);                  \
}

#define RLOGW(Fmt, ...)           \
{                                   \
    IOT_PRINT_RAW(LOG_WARN, Fmt, ##__VA_ARGS__);                  \
}

#define RLOGD(Fmt, ...)           \
{                                   \
    IOT_PRINT_RAW(LOG_DEBUG, Fmt, ##__VA_ARGS__);                  \
}

#define RLOGI(Fmt, ...)           \
{                                   \
    IOT_PRINT_RAW(LOG_INFO, Fmt, ##__VA_ARGS__);                  \
}

