#ifndef LOG_MESSAGE_HPP
#define LOG_MESSAGE_HPP

#include <unordered_map>

constexpr int getAppID() { return 20; }

// Log severity levels
enum LogSeverity
{
    INFO = 0,
    ACTIVITY = 1,
    WARNING = 2,
    ERROR = 3
};

// Component identifiers
enum LogComponent
{
    RTMC = 2013,
    APP = 2010,
    CAMERA = 2020,
    KWIS_PLC = 2030,
    REMOTE_PLC = 2031
};

// Component state
enum ComponentState
{
    AVAILABLE_ = 0,
    CONNECTED_ = 1,
    RUNNING_ = 2,
    FAULT_ = 3
};

// Log message codes
enum LogCode
{
    // FATAL
    CAM_GRAB_FAIL = 2020011,      // CAM011
    CAM_LOOP_EXCEPTION = 2020012, // CAM012

    // ERROR
    CAM_INITIALIZE_FAIL = 2020014,              // CAM014
    APP_CROP_RESIZE_FAIL = 2010016,             // APP016
    KVP_LICENSE_INIT_FAIL = 2010003,            // KVP003
    APP_RECIPE_NOT_FOUND_OR_EMPTY = 2010015,    // APP015
    CSY_CAMERA_NOT_READY = 2010008,             // CSY008
    SHW_CUTTER_SENSOR_FAIL = 2010003,           // SHW003
    SHW_DANCER_SENSOR_FAIL = 2010009,           // SHW009
    SHW_ENCODER_MOTION_DIFF = 2010090,          // SHW011, SHW004
    SHW_CAMERA_PULSE_FETCH_FAIL = 2010006,      // NA
    APP_TOTAL_PULSE_NOT_PROCESSED = 2010019,    // APP019
    APP_METADATA_SAVE_FAIL = 2010004,           // APP004
    CAM_CAMERA_PULSE_OVERFLOW = 2020013,        // CAM013
    COM_KWIS_CUTTER_PLC_DISCONNECTED = 2020002, // COM002
    UNKNOWN_DETECTION_EXCEPTION = 2010022,      // New from RR logs

    // WARNING

    APP_RESET_DONE =2010031,
    CAM_NODE_SLOW = 2020007,                   // PERF007
    APP_JSON_NOT_SAVED = 2010015,              // APP015 (duplicate code intentional)
    APP_AUTO_BRIGHTNESS_FAILED = 2010017,      // APP017
    APP_BRIGHTNESS_FABRIC_NOT_FOUND = 2010017, // APP017
    APP_DEFECT_JUMP_HANDLED = 2010018,         // APP018
    CUTT_LAST_10_BODY_MISALIGNED = 2030011,    // CUTT011
    SHW_LOOP_NOT_CLOSED = 2010010,             // SHW010
    CAM2_STOPPER_NOT_UPDATING = 2010001,       // NA
    CAM_Q_FILLING_FAST = 2020015,              // CAM015
    LICENSE_EXPIRY_MSG = 2020003,              // KVP003 (warning variant)

    // INFO
    CAM_LOOP_STARTED = 2010009,                // INFO002
    CAM_CALLBACK_DONE = 2010010,               // INFO007
    CAM_LOAD_NETWORK = 2010005,           // INFO003 / AI005
    CAM_THRESHOLD_CHANGED = 2010006,      // AI006
    CAM_DEFECT_SAVE_TOGGLE = 2010008,     // AI008
    CAM_METADATA_SAVED = 2010009,         // AI009
    SYSTEM_ENGAGED = 2010011,                  // INFO011
    SYSTEM_STATE_IDLE = 2010012,               // INFO011
    CUTTING_MC_RUNNING_NO_SYSTEM_ON = 2010013, // INFO010
    DANCER_SENSOR_SYNCED = 2010014,            // INFO008
    INSPECTION_IGNORED = 2010020,              // INFO009
    APP_RECIPE_MODIFIED_SUCCESS = 2010015,     // APP015
    APP_JSON_LOADED = 2010020,            // APP020
    APP_RECIPES_EMPTY = 2010021,          // APP021
    RECIPE_DELETED_SUCCESS=2010022,      //INFO011
    RECIPE_MODIFIED_SUCCESS=2010023,     //INFO012
    RECIPE_CREATED_SUCCESS=2010024,      //INFO013
    INSPECTION_LIGHT_TURNED_OFF=2010025, //INFO014
    INSPECTION_LIGHT_TURNED_ON=2010026,   //INFO015
    ROLL_START_BUTTON_PRESSED=2010027,
    ROLL_END_BUTTON_PRESSED=2010028,
    BARCODE_CONFIG_SAVED=2010029,
    INTERVAL_TASK_LOG_FUNCTION_CALL=2010030

};

// Mappings
inline const std::unordered_map<std::string, LogCode> &getLogCodeMap()
{
    static const std::unordered_map<std::string, LogCode> codeMap = {
        {"CAM011", CAM_GRAB_FAIL},
        {"CAM012", CAM_LOOP_EXCEPTION},
        {"CAM013", CAM_CAMERA_PULSE_OVERFLOW},
        {"CAM014", CAM_INITIALIZE_FAIL},
        {"CAM015", CAM_Q_FILLING_FAST},
        {"PERF007", CAM_NODE_SLOW},

        {"APP001",APP_RESET_DONE},
        {"APP004", APP_METADATA_SAVE_FAIL},
        {"APP015", APP_RECIPE_NOT_FOUND_OR_EMPTY},
        {"APP016", APP_CROP_RESIZE_FAIL},
        {"APP017", APP_AUTO_BRIGHTNESS_FAILED},
        {"APP018", APP_DEFECT_JUMP_HANDLED},
        {"APP019", APP_TOTAL_PULSE_NOT_PROCESSED},
        {"APP020", APP_JSON_LOADED},
        {"APP021", APP_RECIPES_EMPTY},

        {"KVP003", KVP_LICENSE_INIT_FAIL},

        {"CSY008", CSY_CAMERA_NOT_READY},
        {"CUTT011", CUTT_LAST_10_BODY_MISALIGNED},
        {"SHW003", SHW_CUTTER_SENSOR_FAIL},
        {"SHW004", SHW_ENCODER_MOTION_DIFF},
        {"SHW009", SHW_DANCER_SENSOR_FAIL},
        {"SHW010", SHW_LOOP_NOT_CLOSED},
        {"SHW011", SHW_ENCODER_MOTION_DIFF},
        {"COM002", COM_KWIS_CUTTER_PLC_DISCONNECTED},
        {"CAM015", SHW_CAMERA_PULSE_FETCH_FAIL},

        {"INFO002", CAM_LOOP_STARTED},
        {"INFO003", CAM_LOAD_NETWORK},
        {"INFO007", CAM_CALLBACK_DONE},
        {"INFO008", DANCER_SENSOR_SYNCED},
        {"INFO009", INSPECTION_IGNORED},
        {"INFO010", CUTTING_MC_RUNNING_NO_SYSTEM_ON},
        {"INFO011", RECIPE_DELETED_SUCCESS},
        {"INFO012", RECIPE_MODIFIED_SUCCESS},
        {"INFO013", RECIPE_CREATED_SUCCESS},
        {"INFO014", INSPECTION_LIGHT_TURNED_OFF},
        {"INFO015", INSPECTION_LIGHT_TURNED_ON},
        {"INFO016",ROLL_START_BUTTON_PRESSED},
        {"INFO017",ROLL_END_BUTTON_PRESSED},
        {"INFO018",BARCODE_CONFIG_SAVED},
        {"APP020",INTERVAL_TASK_LOG_FUNCTION_CALL},
        {"INFO011", SYSTEM_ENGAGED},
        {"AI005", CAM_LOAD_NETWORK},
        {"AI006", CAM_THRESHOLD_CHANGED},
        {"AI008", CAM_DEFECT_SAVE_TOGGLE},
        {"AI009", CAM_METADATA_SAVED}};
    return codeMap;
}

inline const std::unordered_map<std::string, LogSeverity> &getLogSeverityMap()
{
    static const std::unordered_map<std::string, LogSeverity> severityMap = {
        {"INFO", INFO},
        {"ACTIVITY", ACTIVITY},
        {"WARNING", WARNING},
        {"ERROR", ERROR}};
    return severityMap;
}

inline LogCode getLogCodeFromString(const std::string &codeStr)
{
    const auto &map = getLogCodeMap();
    auto it = map.find(codeStr);
    if (it != map.end())
        return it->second;
    return static_cast<LogCode>(-1); // Unknown
}

inline LogSeverity getLogSeverityFromString(const std::string &severityStr)
{
    const auto &map = getLogSeverityMap();
    auto it = map.find(severityStr);
    if (it != map.end())
        return it->second;
    return static_cast<LogSeverity>(-1); // Unknown
}

#endif // LOG_MESSAGE_HPP
