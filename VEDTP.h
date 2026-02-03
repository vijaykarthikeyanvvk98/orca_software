/*
 * vedtp.h
 *
 *  Created on: Jun 16, 2025
 *      Author: Sooraj R       // Change this and prepare to die
 */
#include <stdint.h>
#include <string.h>
#include <math.h>
//#include <stdlib.h>
#ifndef VEDTP_VEDTP_H_
#define VEDTP_VEDTP_H_

#define PROTOCOL_VERSION 2

// Payload structures size definitions
#define VEDTP_HEADER_SIZE          107
#define HEARTBEAT_SIZE             13
#define AHRS_SIZE                  10
#define GPS_SIZE                   39
#define IMU_SIZE                   46
#define INTERNAL_ATMOSPHERE_SIZE   17
#define EXTERNAL_ATMOSPHERE_SIZE   38
#define SONAR_SIZE                 12
#define MOTOR_SIZE                 84
#define DEAD_RECKONING_SIZE        40
#define POWER_HEALTH_SIZE          20
#define FC_HEALTH_SIZE             24
#define COMMAND_SIZE               44
#define JOYSTICK_SIZE              22
#define FLOWMETER_SIZE             17
#define COMMAND_ACK_SIZE           58
#define DEVICE_ERROR_SIZE          8
#define CPT_DATA_SIZE              58
#define WAYPOINT_SIZE              44
#define WP_ACK_SIZE                7

// Float convertion scales
#define f_e2 100.0
#define f_e3 1000.0
#define f_e7 10000000.0

/*float → int16 e3 with clamp */
#define F_TO_I16_E3(v) ((v) > 32.767f ? 32767 : ((v) < -32.768f ? -32768 : (int16_t)((v) * 1000.0f)))



// This will be our final structure for VEDTP data packets, we will slowly migrate to this
// #pragma pack(push, 1)
// typedef struct {
//     uint8_t sync1;               // first start byte
//     uint8_t sync2;               // second start byte

//     uint8_t protocol_version;    // protocol version
//     uint8_t encrypt_flag;        // 0 = no encryption, 1 = encrypted
//     uint8_t encryption_iv[12];   // Initialization Vector for encryption
//     uint8_t encrypt_length;      // Length of encrypted payload

//     uint8_t src_sysid;           // Source system ID
//     uint8_t src_compid;          // Source component ID
//     uint8_t dst_sysid;           // Destination system ID

//     uint32_t sequence_number;    // anti replay protection
//     uint8_t payload_length;      // Length of the payload
//     uint8_t payload[100];        // Payload data

//     uint32_t crc32;              // CRC32 for data integrity
//     uint8_t signature[32];       // HMAC-SHA256 signature

// } VEDTP_Main;
// #pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint8_t startByte1; // Start byte for VEDTP
    uint8_t startByte2; // Start byte for VEDTP
    uint8_t protocol_version;
    uint8_t vehicle;
    uint8_t device;
    uint8_t payload_length;
    uint8_t payload[100];
    uint8_t checksum;
} VEDTP_Main;
#pragma pack(pop)

//Protocol Message Structure
#pragma pack(push, 1)
typedef struct {
    uint8_t loggedState;
    uint8_t vehicleID;
    uint8_t armedState;
    uint8_t systemMode;
    uint8_t gps_fix;
    uint8_t failsafe_flags;
    uint8_t system_health;
    uint16_t sensors_validity;
    uint32_t uptime_ms;             // Controller tick in milli seconds (aprox 49 days, 17 hours, 2 minutes, and 47 seconds)
} HEARTBEAT_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    uint32_t uptime_ms;
} AHRS_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct   {
    uint32_t tow_ms;                // GPS time of week
    double  lat_deg;                // latitude * 1e7
    double  lon_deg;                // longitude * 1e7
    float  alt_m;                // altitude MSL
    float ground_speed_mps;   // ground speed in m/s
    float heading_deg;        // gps course in degree
    float hAcc_mm;            // horizontal accuracy in mm
    float vAcc_mm;            // vertical accuracy in mm
    uint32_t age_ms;                // gps age in ms
    uint8_t  fix_type;              // enum: GPSFixStatus
    uint8_t  sats;                  // satellite count
    uint8_t  flags;                 // will implement in future
    uint32_t uptime_ms;             // Controller tick in milli seconds
} GPS_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    float acceleration[3];
    float gyro[3];
    float magnetic[3];
    float rotationvector[3];
    float linearacceleration[3];
    float gravity[3];
    float temperature_C;
    uint8_t systemcalibration;
    uint8_t gyrocalibration;
    uint8_t accelerometercalibration;
    uint8_t magnetometercalibration;
    uint32_t uptime_ms;       // for synchronization and debugging
} IMU_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    float internal_temp_C;        // °C
    float internal_humidity_RH;   // % RH
    float internal_pressure_mbar;  // mbar
    float internal_tvoc_ppb;      // VOC/gas detection
    float internal_eco2_ppm;      // battery off-gas/smoke indicator
    uint8_t leak_status;
    uint32_t uptime_ms;       // for synchronization and debugging
} INTERNAL_ATMOSPHERE_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    // Universal
    float temperature_C;         // Air/Water temp
    float pressure_mbar;          // Barometric or depth-based
    float humidity_RH;           // Only valid in air/land/surface
    float altitude_m;            // Derived from pressure (air/land only)

    // AQ / Pollution (surface/air)
    float tvoc_ppb;              // Total VOCs
    float eco2_ppm;              // CO₂ equiEXTEvalent
    uint16_t air_quality_index;  // AQI score, optional

    // Marine / Underwater
    float depth_m;               // From pressure sensor (e.g., MS5837)
    float salinity_ppt;          // Parts per thousand (marine salinity)
    float conductivity_uS;       // Optional: more accurate than salinity
    float water_dissolved_oxygen_mgL;  // Optional for aquatic monitoring
    float turbidity_NTU;         // Optional: underwater vision/dirtiness

    // Status
    uint8_t sensor_status_flags; // Bitfield: sensor errors, low range, etc.
    uint8_t medium_type;         // 0 = unknown, 1 = air, 2 = water
    uint32_t uptime_ms;       // for synchronization and debugging
} EXTERNAL_ATMOSPHERE_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    uint8_t sonar_type;         // From SonarType enum
    uint8_t sonar_id;           // In case of multi-sonar systems

    // Universal values
    float range_m;              // For 1D/obstacle/multibeam min
    float signal_strength;      // 0.0 to 1.0
    float confidence;           // 0.0 to 1.0

    uint32_t uptime_ms;       // for synchronization and debugging
} SONAR_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    float rpm[4];           // 8B: Rotational speed
    float velocity[4];      // 8B: Linear speed (m/s)
    float distance[4];      // 8B: Total distance (m)
    float current[4];       // 8B: Current draw (A)
    float temperature[4];   // 8B: Motor or ESC temp (°C)
    float torque_est[4];    // 8B: Estimated torque (Nm) or effort
    float voltage[4];       // 8B: Per-motor bus voltage (V)
    float pwm[4];         // 4B: PWM value sent (-1000 to +1000)
    uint8_t fault_code[4];  // 4B: Fault flags (bitmask or code)
    uint8_t control_mode[4]; // 2B: Mode (manual, closed-loop, coast, etc.)
    uint8_t status_flags[4]; // 2B: Sensor OK, temp warn, overcurrent, etc.
    uint8_t reserved[4];     // 2B: Padding/future flags
    uint32_t uptime_ms;       // for synchronization and debugging
} MOTOR_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    int32_t x_mm;                 // Local X (meters)
    int32_t y_mm;                 // Local Y (meters)
    int32_t z_mm;                 // Local Z or altitude (meters)

    float yaw_cdeg;               // Fused yaw
    float speed_mps;             // Fused ground speed
    // GPS data
    double gps_lat;           // Degrees
    double gps_lon;
    double gps_alt;           // MSL or ellipsoid
    float gps_speed;         // m/s
    float gps_course;        // deg
    // Fusion state
    uint8_t gps_fused;       // 1 = used in EKF, 0 = not
    uint8_t deadrec_active;  // 1 = dead reckoning engaged
    uint8_t gps_valid;       // 1 = GPS fix OK
    uint8_t reserved;        // Alignment

    uint32_t uptime_ms;       // for synchronization and debugging
} DEAD_RECKONING_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    float smps_voltage;        // Output of SMPS
    float battery_voltage;     // Battery pack voltage
    float current_draw;        // Total current drawn from battery or SMPS (A)
    float battery_capacity_pct;// 0.0–100.0% (if fuel gauge or estimated)

    float power_usage_watt;    // Power = voltage * current (W)
    float estimated_uptime_min;// Time remaining if discharging (minutes)

    uint8_t charging;          // 1 = charging, 0 = not
    uint8_t discharging;       // 1 = active draw from battery
    uint8_t battery_present;   // 1 = battery connected
    uint8_t power_flags;       // Bitfield: overvolt, undervolt, fail, etc.

    uint32_t uptime_ms;       // for synchronization and debugging
} POWER_HEALTH_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    float temperature;           // °C (from MCU or IMU)
    float cpu_usage_pct;         // % total CPU usage (RTOS-aware)
    float heap_usage_pct;        // % of heap used (dynamic RAM)
    float stack_high_watermark_pct; // Min % of available stack across tasks

    uint32_t tick_count;         // RTOS tick count

    uint16_t error_count;        // Fatal/critical errors
    uint16_t warning_count;      // Warnings or soft faults

    uint8_t reset_reason;        // From MCU flags
    uint8_t rtos_ok;             // 1 = scheduler running, 0 = fault
    uint8_t task_fault_flags;    // Bitfield for specific task failures
    uint8_t failsafe_flags;      // Bitfield for mission-level failsafe
    uint32_t uptime_ms;       // for synchronization and debugging
} FC_HEALTH_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    uint8_t  id;               // Command ID (enum or type)
    uint8_t  target;           // Target system or submodule (0 = FC, 1 = NAV, etc.)
    uint8_t  flags;            // Bitfield: ACK required, retry, priority
    uint8_t  param_count;      // How many params are valid

    int16_t  param[6];         // Signed 16-bit params for flexibility (-32768 to 32767)

    char     char1[12];        // Label, key, mode name, etc.
    char     char2[12];        // Optional second string

    uint32_t uptime_ms;       // for synchronization and debugging

} COMMAND_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    uint16_t channels[8];    // CH1 to CH8 (1000–2000 or 0–4095)
    uint8_t  channel_count;  // Active channels (e.g., 6 or 8)
    uint8_t  flags;          // Bitmask: failsafe, override, armed, etc.
    uint32_t uptime_ms;       // for synchronization and debugging
} JOYSTICK_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    float rpm;               // Raw RPM from prop or impeller
    float flow_speed_mps;    // Calculated flow speed in m/s
    float flow_direction_deg;// Direction of flow (0–360°)

    float roll;              // Orientation of sensor (degrees)
    float pitch;
    float yaw;

    uint8_t status_flags;    // Bitmask for sensor OK, calibration, data validity
    uint32_t uptime_ms;       // for synchronization and debugging
} FLOWMETER_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    uint8_t  command;        // Command ID this ACK refers to
    uint8_t  result;         // See ResultCode enum below
    uint8_t  source;         // Who responded (e.g. FC, NAV, CAM)
    uint8_t  flags;          // Optional: retry, delayed, etc.
    char     message[50];    // Optional: "OK", "Param error", "Timed out"
    uint32_t uptime_ms;       // for synchronization and debugging
} COMMAND_ACK_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    uint8_t device_id;   // Which device (AHRS, GPS, etc.)
    uint8_t error_type;  // Type of fault (init, data, timeout, etc.)
    uint8_t severity;    // 0 = Info, 1 = Warning, 2 = Critical
    uint8_t flags;       // Bitfield: persistent, auto-reset, etc.
    uint32_t uptime_ms;       // for synchronization and debugging
} DEVICE_ERROR_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    // Core CPT metrics
    float angle_deg;            // CPT arm/rod tilt
    float pressure_mbar;         // Penetration resistance (mbar)
    int16_t penetration_mm;        // Depth into sediment (mm)
    float insertion_speed_mmps;  // Real-time penetration rate
    float strain_raw;           // Raw ADC strain or loadcell
    float rod_temp_c;           // Load cell or probe temp

    // Position and orientation (for mapping)
    double latitude_deg;         // WGS84
    double longitude_deg;
    float depth_m;              // Depth of vehicle in water

    float roll_deg;             // Orientation of vehicle or arm
    float pitch_deg;
    float yaw_deg;
    // System status and health
    uint8_t  status_flags;      // CPT-specific flags (see below)
    uint8_t  soil_class;        // Estimated soil type (0 = unknown, 1 = clay, etc.)
    uint8_t  step_index;        // For multi-point profiles (0 = init, >0 = next)
    uint8_t  active;            // 1 = active probe in progress

    char     label[16];         // Optional tag: "POINT_A", "AUTO_RUN", etc.
    uint32_t uptime_ms;       // for synchronization and debugging
} CPT_DATA_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    uint8_t  id;               // Current waypoint ID
    uint8_t  wp_count;         // Total waypoints in mission
    uint8_t  wp_type;          // Type: NAV, DIVE, SURVEY, etc.
    uint8_t  next_cmd;         // Next behavior: STOP, CONTINUE, RTL, etc.
    uint8_t  action_code;      // Trigger action: drop, relay, sample, etc.
    uint8_t  frame;            // Coordinate frame: 0=Global, 1=Local, 2=NED
    double  lat_deg;          // WGS84 latitude
    double  lon_deg;          // WGS84 longitude
    int32_t  alt_mm;            // Altitude/Depth (mm)
    float  speed_mps;        // Desired transit speed
    float  radius_m;         // Acceptance radius (m)
    int16_t  hold_time_s;      // Hold or wait duration (sec)
    float  yaw_deg;          // Desired heading at waypoint
    uint8_t  flags;            // Optional: bitmask for options (loiter, one-shot, etc.)
    uint8_t  retries;          // If waypoint fails, how many retries?
    char     label[12];        // Optional waypoint name (for logs or UI)
    uint32_t uptime_ms;       // for synchronization and debugging
} WAYPOINT_Packet;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct  {
    uint16_t wp_id;
    uint8_t result; // 0 = OK, 1 = CRC fail, 2 = invalid index, etc.
    uint32_t uptime_ms;       // for synchronization and debugging
} WP_ACK_Packet;
#pragma pack(pop)


#define MAX_BUFFER_SIZE 256


typedef enum {
    VEDTP_INCOMPLETE = 0,
    VEDTP_COMPLETE,
    VEDTP_INVALID_START_BYTES,
    VEDTP_INVALID_SECOND_BYTES,
    VEDTP_CRC_FAIL,
    VEDTP_PROTOCOL_VERSION_MISMATCH
} VEDTP_ParseResult;

typedef enum {
    NOT_LOGGED = 0,
    LOGGED,
}Logged_State;

typedef enum {
    VEHICLE_ACR = 1,		// Amphibious Crawler Rover
    VEHICLE_ROV6,           // ROV with 6 thrusters
    VEHICLE_ROV8,           // ROV with 8 thrusters
    VEHICLE_ASV,            // Autonomous Surface Vehicle
    VEHICLE_MINIROV,        // Mini ROV for small tasks
    VEHICLE_AUV,            // Autonomous Underwater Vehicle
    VEHICLE_DRONE,          // Drone for aerial tasks
    VEHICLE_CUSTOM = 0xFF   // If ROV wants wings
} Vehicle_ID;

typedef enum{
    DISARMED = 0,
    ARMED,
}ARM_State;

typedef enum {
    MODE_HOLD = 0,
    MODE_MANUAL,
    MODE_DEPTH_HOLD,
    MODE_STABILIZE,
    MODE_POSITION_HOLD,
    MODE_ALTITUDE_HOLD,
    MODE_VELOCITY_HOLD,
    MODE_COURSE_HOLD,
    MODE_WAYPOINT_NAVIGATION,
    MODE_SURVEY,
    MODE_DIVE,
    MODE_SURFACE,
    MODE_MISSION,
    MODE_GUIDED,
    MODE_AUTO,
    MODE_RTL,
    MODE_FOLLOW,
} System_Mode;


typedef enum {
    DEVICE_HEARTBEAT = 1,
    DEVICE_GPS,
    DEVICE_AHRS,
    DEVICE_IMU,
    DEVICE_INTERNAL_ATMOSPHERE,
    DEVICE_EXTERNAL_ATMOSPHERE,
    DEVICE_SONAR,
    DEVICE_MOTOR,
    DEVICE_DEAD_RECKONING,
    DEVICE_POWER_HEALTH,
    DEVICE_FC_HEALTH,
    DEVICE_COMMAND,
    DEVICE_JOYSTICK,
    DEVICE_FLOW_METER,
    DEVICE_COMMAND_ACK,
    DEVICE_DEVICE_ERROR,
    DEVICE_CPT_DATA,
    DEVICE_WAYPOINT,
    DEVICE_WP_ACK,
} DeviceID;

typedef enum {
    ACK_GOOD = 1,
    ACK_LOGIN_ERROR,
    ACK_LOGIN_SUCCESS,
    ACK_COMMAND_ERROR,
    ACK_COMMAND_SUCCESS
} AckResult;

typedef enum {
    CMD_LOGIN = 0,
    CMD_ARM_DISARM,
    CMD_MODE,
    CMD_BRAKE,
    CMD_CPT,
    CMD_DISCONNECT,
    CMD_SET_RTL_COORDS,

    CMD_SELF_DESTRUCT = 255
} CommandID;


typedef enum {
    DEVICE_ERROR_INIT = 1,
    DEVICE_ERROR_HEARTBEAT,
    DEVICE_ERROR_DATA_TRANSFER_FAILED,
    DEVICE_ERROR_CRC_MISMATCH,
    DEVICE_ERROR_TIMEOUT,
    DEVICE_ERROR_OUT_OF_RANGE,
    DEVICE_ERROR_NOT_RESPONDING,
    DEVICE_ERROR_COMMUNICATION_LOST,
    DEVICE_ERROR_CALIBRATION_FAILED,
    DEVICE_ERROR_UNKNOWN
} DeviceErrorType;

#define Freq(Hz) (round(1000.0 / (Hz)))

typedef enum {
    FREQ_HEARTBEAT = 1,
    FREQ_AHRS = 3,
    FREQ_GPS = 1,
    FREQ_IMU = 2,
    FREQ_INTERNAL_ATMOSPHERE = 2,
    FREQ_EXTERNAL_ATMOSPHERE = 2,
    FREQ_SONAR = 2,
    FREQ_MOTOR = 3,
    FREQ_DEAD_RECKONING = 1,
    FREQ_POWER_HEALTH  = 2,
    FREQ_FC_HEALTH = 2,
    FREQ_FLOW_SENSOR = 2,
    FREQ_JOYSTICK = 10,
    FREQ_CPT_DATA = 5
} MessageFrequencyHz;

typedef enum {
    GPS_NO_FIX = 0,
    GPS_2D_FIX = 2,
    GPS_3D_FIX = 3,
    GPS_DGPS_FIX = 4,
    GPS_RTK_FLOAT = 5,
    GPS_RTK_FIXED = 6
} GPSFixStatus;

typedef enum {
    FAILSAFE_NONE = 0,
    FAILSAFE_GPS_LOSS = 1 << 0, // GPS signal lost
    FAILSAFE_RC_LOSS = 1 << 1,  // RC signal lost
    FAILSAFE_LOW_BATTERY = 1 << 2, // Battery below threshold
    FAILSAFE_SYSTEM_ERROR = 1 << 3, // Critical system error
    FAILSAFE_OVERHEAT = 1 << 4, // Overheating detected
    FAILSAFE_UNDERHEAT = 1 << 5, // Underheating detected
    FAILSAFE_SENSOR_FAILURE = 1 << 6, // Sensor failure detected
    FAILSAFE_CUSTOM = 1 << 7 // Custom user-defined failsafe
}Failsafe_flags;

typedef enum {

    SYSTEM_HEALTH_OK = 0, // All systems nominal
    SYSTEM_HEALTH_WARN,
    SYSTEM_HEALTH_ERROR,
    SYSTEM_HEALTH_CRITICAL,
    SYSTEM_HEALTH_MAINTENANCE,
    SYSTEM_HEALTH_UNKNOWN
}SystemHealth;


uint8_t feed_Me_Bytes(uint8_t rxByte, VEDTP_Main *storage);
uint8_t calculateChecksum(const uint8_t *data, uint8_t length);
uint8_t parseData(uint8_t *incoming, VEDTP_Main *storage);
void vedtp_pack_data(VEDTP_Main *data);


static inline void put_u16_le(uint8_t *b, uint16_t o, uint16_t v);
static inline void put_i16_le(uint8_t *b, uint16_t o, int16_t v);
static inline void put_u32_le(uint8_t *b, uint16_t o, uint32_t v);
static inline void put_i32_le(uint8_t *b, uint16_t o, int32_t v);
static inline uint16_t get_u16_le(const uint8_t *b, uint16_t o);
static inline int16_t get_i16_le(const uint8_t *b, uint16_t o);
static inline uint32_t get_u32_le(const uint8_t *b, uint16_t o);
static inline int32_t get_i32_le(const uint8_t *b, uint16_t o);

uint8_t encode_HEARTBEAT(VEDTP_Main *out, const HEARTBEAT_Packet *data);
uint8_t encode_AHRS(VEDTP_Main *out, const AHRS_Packet *data);
uint8_t encode_GPS(VEDTP_Main *out, const GPS_Packet *data);
uint8_t encode_IMU(VEDTP_Main *out, const IMU_Packet *data);
uint8_t encode_INTERNAL_ATMOSPHERE( VEDTP_Main *out, const INTERNAL_ATMOSPHERE_Packet *data);
uint8_t encode_EXTERNAL_ATMOSPHERE( VEDTP_Main *out, const EXTERNAL_ATMOSPHERE_Packet *data);
uint8_t encode_SONAR( VEDTP_Main *out, const SONAR_Packet *data);
uint8_t encode_MOTOR(VEDTP_Main *out, const MOTOR_Packet *data);
uint8_t encode_DEAD_RECKONING( VEDTP_Main *out, const DEAD_RECKONING_Packet *data);
uint8_t encode_POWER_HEALTH( VEDTP_Main *out, const POWER_HEALTH_Packet *data);
uint8_t encode_FC_HEALTH( VEDTP_Main *out, const FC_HEALTH_Packet *data);
uint8_t encode_COMMAND( VEDTP_Main *out, const COMMAND_Packet *data);
uint8_t encode_JOYSTICK( VEDTP_Main *out, const JOYSTICK_Packet *data);
uint8_t encode_FLOWMETER( VEDTP_Main *out, const FLOWMETER_Packet *data);
uint8_t encode_COMMAND_ACK( VEDTP_Main *out, const COMMAND_ACK_Packet *data);
uint8_t encode_DEVICE_ERROR( VEDTP_Main *out, const DEVICE_ERROR_Packet *data);
uint8_t encode_CPT_DATA( VEDTP_Main *out, const CPT_DATA_Packet *data);
uint8_t encode_WAYPOINT( VEDTP_Main *out, const WAYPOINT_Packet *data);
uint8_t encode_WP_ACK( VEDTP_Main *out, const WP_ACK_Packet *data);

uint8_t decode_HEARTBEAT(const VEDTP_Main *vedtp, HEARTBEAT_Packet *data);
uint8_t decode_AHRS(const VEDTP_Main *vedtp, AHRS_Packet *data);
uint8_t decode_GPS(const VEDTP_Main *vedtp, GPS_Packet *data);
uint8_t decode_IMU(const VEDTP_Main *vedtp, IMU_Packet *data);
uint8_t decode_INTERNAL_ATMOSPHERE( const VEDTP_Main *vedtp, INTERNAL_ATMOSPHERE_Packet *data);
uint8_t decode_EXTERNAL_ATMOSPHERE( const VEDTP_Main *vedtp, EXTERNAL_ATMOSPHERE_Packet *data);
uint8_t decode_SONAR(const VEDTP_Main *vedtp, SONAR_Packet *data);
uint8_t decode_MOTOR(const VEDTP_Main *vedtp, MOTOR_Packet *data);
uint8_t decode_DEAD_RECKONING( const VEDTP_Main *vedtp, DEAD_RECKONING_Packet *data);
uint8_t decode_POWER_HEALTH( const VEDTP_Main *vedtp, POWER_HEALTH_Packet *data);
uint8_t decode_FC_HEALTH( const VEDTP_Main *vedtp, FC_HEALTH_Packet *data);
uint8_t decode_COMMAND( const VEDTP_Main *vedtp, COMMAND_Packet *data);
uint8_t decode_JOYSTICK( const VEDTP_Main *vedtp, JOYSTICK_Packet *data);
uint8_t decode_FLOWMETER( const VEDTP_Main *vedtp, FLOWMETER_Packet *data);
uint8_t decode_COMMAND_ACK( const VEDTP_Main *vedtp, COMMAND_ACK_Packet *data);
uint8_t decode_DEVICE_ERROR( const VEDTP_Main *vedtp, DEVICE_ERROR_Packet *data);
uint8_t decode_CPT_DATA( const VEDTP_Main *vedtp, CPT_DATA_Packet *data);
uint8_t decode_WAYPOINT( const VEDTP_Main *vedtp, WAYPOINT_Packet *data);
uint8_t decode_WP_ACK( const VEDTP_Main *vedtp, WP_ACK_Packet *data);



#endif /* VEDTP_VEDTP_H_ */
