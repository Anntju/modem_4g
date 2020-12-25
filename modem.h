

typedef enum {
    RADIO_STATE_OFF = 0,                   /* Radio explictly powered off (eg CFUN=0) */
    RADIO_STATE_UNAVAILABLE = 1,           /* Radio unavailable (eg, resetting or not booted) */
    RADIO_STATE_ON = 10                    /* Radio is on */
} RIL_RadioState;

typedef struct {
    int signalStrength;  /* Valid values are (0-31, 99) as defined in TS 27.007 8.5 */
    int bitErrorRate;    /* bit error rate (0-7, 99) as defined in TS 27.007 8.5 */
} RIL_SignalStrength;

typedef enum {
    RIL_NOT_REG_AND_NOT_SEARCHING = 0,           // Not registered, MT is not currently searching
                                                 // a new operator to register
    RIL_REG_HOME = 1,                            // Registered, home network
    RIL_NOT_REG_AND_SEARCHING = 2,               // Not registered, but MT is currently searching
                                                 // a new operator to register
    RIL_REG_DENIED = 3,                          // Registration denied
    RIL_UNKNOWN = 4,                             // Unknown
    RIL_REG_ROAMING = 5,                         // Registered, roaming
} RIL_RegState;

typedef enum {
    SIM_ABSENT = 0,
    SIM_NOT_READY = 1,
    SIM_READY = 2,
    SIM_PIN = 3,
    SIM_PUK = 4,
    SIM_NETWORK_PERSONALIZATION = 5,
    SIM_FAILURE = 6,
} SIM_Status;

typedef enum {
    POLL_OK = 0,
    POLL_FAILD = 1,
    POLL_ERROR_SIM = 2,
} POLL_Status;

