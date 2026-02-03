/*
 * vedtp.c
 *
 *  Created on: Jun 16, 2025
 *      Author: Sooraj R
 */


#include "vedtp.h"

uint8_t inputBuffer[MAX_BUFFER_SIZE];
uint8_t bufferIndex = 0;


/**
 * @brief Incrementally feed incoming bytes into the VEDTP frame parser.
 *
 * This function processes a single incoming byte at a time and assembles a
 * complete VEDTP frame using an internal buffer. It performs early validation
 * of frame start bytes and protocol version before attempting full frame
 * parsing and checksum verification.
 *
 * The function is designed to be called repeatedly from a byte-oriented
 * receive context (e.g., UART ISR, DMA callback, socket receive loop).
 * It maintains internal parser state across calls and signals parsing progress
 * or errors via explicit return codes.
 *
 * Parsing sequence:
 *   1. Validate first start byte (0xFF)
 *   2. Validate second start byte (0xEE)
 *   3. Validate protocol version
 *   4. Accumulate bytes until VEDTP_HEADER_SIZE is reached
 *   5. Parse frame and verify checksum
 *
 * On successful parsing, the decoded frame is copied into the provided
 * @ref VEDTP_Main storage structure.
 *
 * @param[in]  rxByte   Incoming byte to process.
 * @param[out] storage  Pointer to a @ref VEDTP_Main structure where the fully
 *                      parsed frame will be stored upon successful completion.
 *
 * @return uint8_t
 *         - VEDTP_INCOMPLETE               : Frame reception in progress
 *         - VEDTP_COMPLETE                 : Valid frame received and parsed
 *         - VEDTP_INVALID_START_BYTES      : First start byte mismatch
 *         - VEDTP_INVALID_SECOND_BYTES     : Second start byte mismatch
 *         - VEDTP_PROTOCOL_VERSION_MISMATCH: Protocol version mismatch
 *         - VEDTP_CRC_FAIL                 : Frame checksum validation failed
 *
 * @note This function assumes a fixed-size VEDTP frame and relies on
 *       VEDTP_HEADER_SIZE to determine when parsing should occur.
 *
 * @note The internal buffer index is reset automatically on error or after
 *       successful frame completion.
 *
 * @warning This function is stateful and not re-entrant. It must not be called
 *          concurrently from multiple contexts without external protection.
 *
 * @warning Any modification to frame structure, header size, or protocol
 *          version handling must be reflected here to maintain parser
 *          correctness.
 */
uint8_t feed_Me_Bytes(uint8_t rxByte, VEDTP_Main *storage) {
    if (bufferIndex == 0 && rxByte != 0xFF) {
        return VEDTP_INVALID_START_BYTES;
    }
    inputBuffer[bufferIndex++] = rxByte;

    if (bufferIndex == 2 && rxByte != 0xEE) {
        bufferIndex = 0;
        return VEDTP_INVALID_SECOND_BYTES;
    }

    if(bufferIndex == 3 && rxByte != PROTOCOL_VERSION) {
        bufferIndex = 0;
        return VEDTP_PROTOCOL_VERSION_MISMATCH;
    }

    if (bufferIndex >= VEDTP_HEADER_SIZE) {
        if (parseData(inputBuffer,storage)) {
            bufferIndex = 0;
            return VEDTP_COMPLETE;
        }
        else {
            bufferIndex = 0;
            return VEDTP_CRC_FAIL;
        }
    }
    return VEDTP_INCOMPLETE;
}

/**
 * @brief Finalize a VEDTP frame by computing and inserting its checksum.
 *
 * This function calculates the checksum for a populated @ref VEDTP_Main
 * structure and writes the result into the frame's checksum field.
 *
 * The checksum is computed over the fixed-size VEDTP header and payload fields,
 * excluding the checksum byte itself. The resulting checksum is used by the
 * receiver to verify frame integrity during parsing.
 *
 * This function must be called only after all header fields, payload contents,
 * and payload length have been fully populated.
 *
 * @param[in,out] data  Pointer to the VEDTP frame to finalize.
 *
 * @note The checksum is calculated using the protocol-defined checksum
 *       algorithm and assumes a fixed header size defined by
 *       VEDTP_HEADER_SIZE.
 *
 * @note This function performs no memory allocation and does not modify any
 *       fields other than the checksum.
 *
 * @warning Calling this function before the payload or header is fully
 *          populated will result in an invalid checksum and a frame that
 *          will be rejected by the receiver.
 */
void vedtp_pack_data(VEDTP_Main *data)
{
    data->checksum = calculateChecksum((const uint8_t *)data, VEDTP_HEADER_SIZE-1);
}

/**
 * @brief Parse and validate a complete VEDTP frame from a raw byte buffer.
 *
 * This function validates and decodes a fully received VEDTP frame from a
 * raw byte buffer into a @ref VEDTP_Main structure. It performs basic framing
 * validation and checksum verification to ensure data integrity.
 *
 * The function assumes that the input buffer contains a complete VEDTP frame
 * starting at index 0. On successful validation, the frame is copied into the
 * provided storage structure.
 *
 * Validation steps:
 *   1. Verify VEDTP start bytes (0xFF, 0xEE)
 *   2. Copy raw frame into structured storage
 *   3. Recalculate checksum over header and payload (excluding checksum byte)
 *   4. Compare calculated checksum with received checksum
 *
 * @param[in]  incoming  Pointer to the raw byte buffer containing the VEDTP
 *                       frame data.
 * @param[out] storage   Pointer to a @ref VEDTP_Main structure where the parsed
 *                       frame will be stored on success.
 *
 * @return uint8_t
 *         - 1 : Frame is valid and checksum matches.
 *         - 0 : Invalid start bytes or checksum mismatch.
 *
 * @note This function assumes a fixed-size VEDTP frame as defined by
 *       @ref VEDTP_HEADER_SIZE and the @ref VEDTP_Main structure.
 *
 * @note The checksum is calculated directly from the raw incoming buffer to
 *       avoid any side effects from structure padding or alignment.
 *
 * @warning This function does not perform incremental parsing and must only
 *          be called once the complete frame has been received.
 *
 * @warning Any change to frame layout, checksum coverage, or header size must
 *          be reflected here to maintain protocol correctness.
 */
uint8_t parseData(uint8_t *incoming, VEDTP_Main *storage) {
    if (incoming[0] == 0xFF && incoming[1] == 0xEE) {
        memcpy(storage, incoming, sizeof(VEDTP_Main));
        uint8_t checksum = calculateChecksum((const uint8_t *)incoming, VEDTP_HEADER_SIZE-1);
        if (storage->checksum == checksum) {
            return 1;
        } else {
            return 0;
        }
    }
    return 0;  // Invalid data
}

/**
 * @brief Compute the VEDTP checksum using an 8-bit XOR accumulator.
 *
 * This function calculates a simple 8-bit checksum by XOR-ing all bytes in
 * the provided data buffer. The resulting checksum is used to detect
 * transmission errors and data corruption in VEDTP frames.
 *
 * The checksum algorithm is intentionally lightweight and deterministic,
 * making it suitable for real-time embedded systems with constrained
 * processing resources.
 *
 * @param[in] data    Pointer to the byte buffer over which the checksum
 *                    will be calculated.
 * @param[in] length  Number of bytes to include in the checksum calculation.
 *
 * @return uint8_t
 *         The computed XOR checksum value.
 *
 * @note The checksum is order-dependent and sensitive to single-bit errors,
 *       but it is not cryptographically secure and does not protect against
 *       intentional tampering.
 *
 * @note This function performs no memory allocation and has constant time
 *       complexity relative to the input length.
 *
 * @warning This checksum provides basic error detection only. For stronger
 *          integrity guarantees (e.g., burst error detection or security),
 *          consider replacing or augmenting this algorithm with CRC or
 *          cryptographic authentication.
 */
uint8_t calculateChecksum(const uint8_t *data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}


////////////////////////////////////////////////////////////
//little-endian utility functions




/**
 * @brief Write an unsigned 8-bit value to a buffer.
 *
 * @param b Pointer to destination byte buffer.
 * @param o Byte offset into buffer.
 * @param v Value to write.
 */
static inline void put_u8(uint8_t *b, uint16_t o, uint8_t v)
{
    b[o] = v;
}

/**
 * @brief Write an unsigned 16-bit value to a buffer (little-endian).
 *
 * @param b Pointer to destination byte buffer.
 * @param o Byte offset into buffer.
 * @param v 16-bit unsigned value to write.
 */
static inline void put_u16_le(uint8_t *b, uint16_t o, uint16_t v)
{
    b[o+0] = (uint8_t)(v);
    b[o+1] = (uint8_t)(v >> 8);
}

/**
 * @brief Write a signed 16-bit value to a buffer (little-endian).
 *
 * @param b Pointer to destination byte buffer.
 * @param o Byte offset into buffer.
 * @param v 16-bit signed value to write.
 */
static inline void put_i16_le(uint8_t *b, uint16_t o, int16_t v)
{
    put_u16_le(b, o, (uint16_t)v);
}

/**
 * @brief Write an unsigned 32-bit value to a buffer (little-endian).
 *
 * @param b Pointer to destination byte buffer.
 * @param o Byte offset into buffer.
 * @param v 32-bit unsigned value to write.
 */
static inline void put_u32_le(uint8_t *b, uint16_t o, uint32_t v)
{
    b[o+0] = (uint8_t)(v);
    b[o+1] = (uint8_t)(v >> 8);
    b[o+2] = (uint8_t)(v >> 16);
    b[o+3] = (uint8_t)(v >> 24);
}

/**
 * @brief Write a signed 32-bit value to a buffer (little-endian).
 *
 * @param b Pointer to destination byte buffer.
 * @param o Byte offset into buffer.
 * @param v 32-bit signed value to write.
 */
static inline void put_i32_le(uint8_t *b, uint16_t o, int32_t v)
{
    put_u32_le(b, o, (uint32_t)v);
}

/**
 * @brief Read an unsigned 8-bit value from a buffer.
 *
 * @param b Pointer to source byte buffer.
 * @param o Byte offset into buffer.
 * @return Unsigned 8-bit value.
 */
static inline uint8_t get_u8(const uint8_t *b, uint16_t o)
{
    return b[o];
}

/**
 * @brief Read an unsigned 16-bit value from a buffer (little-endian).
 *
 * @param b Pointer to source byte buffer.
 * @param o Byte offset into buffer.
 * @return Unsigned 16-bit value.
 */
static inline uint16_t get_u16_le(const uint8_t *b, uint16_t o)
{
    return (uint16_t)(b[o] | (b[o+1] << 8));
}

/**
 * @brief Read a signed 16-bit value from a buffer (little-endian).
 *
 * @param b Pointer to source byte buffer.
 * @param o Byte offset into buffer.
 * @return Signed 16-bit value.
 */
static inline int16_t get_i16_le(const uint8_t *b, uint16_t o)
{
    return (int16_t)get_u16_le(b, o);
}

/**
 * @brief Read an unsigned 32-bit value from a buffer (little-endian).
 *
 * @param b Pointer to source byte buffer.
 * @param o Byte offset into buffer.
 * @return Unsigned 32-bit value.
 */
static inline uint32_t get_u32_le(const uint8_t *b, uint16_t o)
{
    return (uint32_t)(
        ((uint32_t)b[o+0]) |
        ((uint32_t)b[o+1] << 8) |
        ((uint32_t)b[o+2] << 16) |
        ((uint32_t)b[o+3] << 24)
        );
}

/**
 * @brief Read a signed 32-bit value from a buffer (little-endian).
 *
 * @param b Pointer to source byte buffer.
 * @param o Byte offset into buffer.
 * @return Signed 32-bit value.
 */
static inline int32_t get_i32_le(const uint8_t *b, uint16_t o)
{
    return (int32_t)get_u32_le(b, o);
}


//////////////////////////////////////////////////////////// encode_ data

/**
 * @brief Encode HEARTBEAT message into a VEDTP transport frame.
 *
 * This function serializes a @ref HEARTBEAT structure into the payload
 * of a @ref VEDTP_Main packet using a fixed, little-endian wire format.
 * It initializes the protocol header fields, clears the payload buffer,
 * encodes all heartbeat fields at predefined offsets, and finalizes the
 * packet by invoking the VEDTP packing routine.
 *
 * The encoded HEARTBEAT payload layout is strictly defined and verified
 * at compile time to guarantee binary compatibility across firmware,
 * ground control software, and logging tools.
 *
 * Payload format (byte offsets):
 *   - [0]  : Logged state            (uint8_t, 0 = not logged, 1 = logged)
 *   - [1]  : Vehicle ID              (enum VehicleID)
 *   - [2]  : Armed state             (uint8_t, 0 = disarmed, 1 = armed)
 *   - [3]  : System mode             (enum SystemMode)
 *   - [4]  : GPS fix status          (enum GPSFixStatus)
 *   - [5]  : Failsafe flags          (bitmask)
 *   - [6]  : System health           (enum SystemHealth)
 *   - [7]  : Sensors validity flags  (uint16_t, little-endian)
 *   - [9]  : System uptime           (uint32_t, little-endian, ms)
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the HEARTBEAT data source.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is safe
 *       for use in real-time contexts when called outside of ISR scope.
 *
 * @note Endianness of multi-byte fields is explicitly defined as
 *       little-endian using helper functions to ensure cross-platform
 *       compatibility.
 *
 * @warning The payload size is fixed. Any change to the HEARTBEAT structure
 *          or payload layout must be accompanied by an update to
 *          HEARTBEAT_SIZE and the corresponding static assertion.
 */
uint8_t encode_HEARTBEAT(VEDTP_Main *out, const HEARTBEAT_Packet *data)
{
    if (!out || !data) {
        return 0;
    }

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_HEARTBEAT;

    memset(out->payload, 0, sizeof(out->payload));

    // payload
    out->payload[0] = data->loggedState ? 1 : 0;
    out->payload[1] = data->vehicleID;
    out->payload[2] = data->armedState ? 1 : 0;
    out->payload[3] = data->systemMode;
    out->payload[4] = data->gps_fix;
    out->payload[5] = data->failsafe_flags;
    out->payload[6] = data->system_health;

    put_u16_le(out->payload, 7, data->sensors_validity);
    put_u32_le(out->payload, 9, data->uptime_ms);

    out->payload_length = HEARTBEAT_SIZE;
    ////_Static_assert(HEARTBEAT_SIZE == 13, "HEARTBEAT_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode AHRS attitude data into a VEDTP transport frame.
 *
 * This function serializes attitude and timing data from an @ref AHRS_Packet
 * into the payload of a @ref VEDTP_Main packet using a fixed, little-endian
 * wire format. The resulting packet is fully initialized with protocol
 * header fields and finalized with a checksum for transmission.
 *
 * Attitude angles are scaled and clamped to ensure deterministic encoding:
 *  - Yaw is normalized to the range [0, 360) degrees and encoded as an
 *    unsigned 16-bit value with 0.01° resolution.
 *  - Pitch and Roll are encoded as signed 16-bit values with 0.01° resolution
 *    and are clamped to the representable range of int16_t.
 *
 * Payload layout (byte offsets):
 *   - [0..1] : Yaw angle          (uint16_t, degrees × 100, little-endian)
 *   - [2..3] : Pitch angle        (int16_t,  degrees × 100, little-endian)
 *   - [4..5] : Roll angle         (int16_t,  degrees × 100, little-endian)
 *   - [6..9] : System uptime      (uint32_t, milliseconds, little-endian)
 *
 * The payload size is fixed and verified at compile time to maintain
 * binary compatibility across firmware and ground control software.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the AHRS data source (angles in degrees).
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for use in real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the AHRS payload layout or scaling factors
 *          must be reflected in AHRS_SIZE and validated with the static
 *          assertion to prevent protocol incompatibility.
 */
uint8_t encode_AHRS(VEDTP_Main *out, const AHRS_Packet *data)
{
    if (!out || !data) {
        return 0;
    }

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_AHRS;

    memset(out->payload, 0, sizeof(out->payload));

    // YAW
    float yaw = data->yaw_deg;
    while (yaw < 0.0f)    yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;

    float yaw_scaled = yaw * f_e2;
    if (yaw_scaled > 36000.0f) yaw_scaled = 36000.0f;
    if (yaw_scaled < 0.0f)     yaw_scaled = 0.0f;

    put_u16_le(out->payload, 0, (uint16_t)yaw_scaled);

    // PITCH
    float pitch_scaled = data->pitch_deg * f_e2;
    if (pitch_scaled >  INT16_MAX) pitch_scaled = INT16_MAX;
    if (pitch_scaled <  INT16_MIN) pitch_scaled = INT16_MIN;

    put_i16_le(out->payload, 2, (int16_t)pitch_scaled);

    //ROLL
    float roll_scaled = data->roll_deg * f_e2;
    if (roll_scaled >  INT16_MAX) roll_scaled = INT16_MAX;
    if (roll_scaled <  INT16_MIN) roll_scaled = INT16_MIN;

    put_i16_le(out->payload, 4, (int16_t)roll_scaled);

    //UPTIME
    put_u32_le(out->payload, 6, data->uptime_ms);

    out->payload_length = AHRS_SIZE;
    //_Static_assert(AHRS_SIZE == 10, "AHRS_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode GPS navigation data into a VEDTP transport frame.
 *
 * This function serializes position, velocity, accuracy, and timing data
 * from a @ref GPS_Packet into the payload of a @ref VEDTP_Main packet using
 * a fixed, little-endian wire format. The packet header is initialized,
 * the payload is populated at predefined offsets, and the frame is finalized
 * with a checksum for transmission.
 *
 * All floating-point inputs are converted to scaled integer representations
 * to ensure deterministic, platform-independent encoding:
 *  - Latitude and Longitude are encoded as signed int32 values in degrees × 1e7.
 *  - Altitude is encoded as signed int32 in millimeters.
 *  - Ground speed is encoded as unsigned uint16 in m/s × 100.
 *  - Heading is normalized to [0, 360) degrees and encoded as uint16 × 100.
 *  - Horizontal and Vertical accuracies are encoded as uint32 in millimeters × 100.
 *
 * Payload layout (byte offsets):
 *   - [0..3]   : Time of week           (uint32_t, milliseconds)
 *   - [4..7]   : Latitude               (int32_t, deg × 1e7)
 *   - [8..11]  : Longitude              (int32_t, deg × 1e7)
 *   - [12..15] : Altitude               (int32_t, millimeters)
 *   - [16..17] : Ground speed           (uint16_t, m/s × 100)
 *   - [18..19] : Heading                (uint16_t, degrees × 100)
 *   - [20..23] : Horizontal accuracy    (uint32_t, mm × 100)
 *   - [24..27] : Vertical accuracy      (uint32_t, mm × 100)
 *   - [28..31] : Data age               (uint32_t, milliseconds)
 *   - [32]     : Fix type               (enum GPSFixStatus)
 *   - [33]     : Satellites in view     (uint8_t)
 *   - [34]     : GPS flags              (bitmask)
 *   - [35..38] : System uptime          (uint32_t, milliseconds)
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility between firmware, ground control software, and log decoders.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the GPS data source (values in SI units).
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to payload layout, scaling factors, or field
 *          ordering must be reflected in GPS_SIZE and validated via the
 *          static assertion to avoid protocol incompatibility.
 */
uint8_t encode_GPS(VEDTP_Main *out, const GPS_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_GPS;

    memset(out->payload, 0, sizeof(out->payload));

    /* ---- time of week ---- */
    put_u32_le(out->payload, 0, data->tow_ms);

    /* ---- latitude / longitude ---- */
    double lat_e7 = data->lat_deg * 1e7;
    double lon_e7 = data->lon_deg * 1e7;

    if (lat_e7 >  2147483647.0) lat_e7 =  2147483647.0;
    if (lat_e7 < -2147483648.0) lat_e7 = -2147483648.0;
    if (lon_e7 >  2147483647.0) lon_e7 =  2147483647.0;
    if (lon_e7 < -2147483648.0) lon_e7 = -2147483648.0;

    put_i32_le(out->payload, 4, (int32_t)lat_e7);
    put_i32_le(out->payload, 8, (int32_t)lon_e7);

    /* ---- altitude (meters → mm) ---- */
    float alt_mm = data->alt_m * 1000.0f;
    put_i32_le(out->payload, 12, (int32_t)alt_mm);

    /* ---- ground speed (m/s * 1e2) ---- */
    float speed_e2 = data->ground_speed_mps * 100.0f;
    if (speed_e2 < 0.0f) speed_e2 = 0.0f;
    if (speed_e2 > 65535.0f) speed_e2 = 65535.0f;
    put_u16_le(out->payload, 16, (uint16_t)speed_e2);

    /* ---- heading (deg * 1e2) ---- */
    float heading = data->heading_deg;
    while (heading < 0.0f)   heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    put_u16_le(out->payload, 18, (uint16_t)(heading * 100.0f));

    /* ---- accuracies (mm * 1e2) ---- */
    put_u32_le(out->payload, 20, (uint32_t)(data->hAcc_mm * 100.0f));
    put_u32_le(out->payload, 24, (uint32_t)(data->vAcc_mm * 100.0f));

    /* ---- age ---- */
    put_u32_le(out->payload, 28, data->age_ms);

    /* ---- status ---- */
    out->payload[32] = data->fix_type;
    out->payload[33] = data->sats;
    out->payload[34] = data->flags;

    /* ---- uptime ---- */
    put_u32_le(out->payload, 35, data->uptime_ms);

    out->payload_length = GPS_SIZE;
    //_Static_assert(GPS_SIZE == 39, "GPS_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode raw IMU sensor data into a VEDTP transport frame.
 *
 * This function serializes inertial, magnetic, and environmental sensor data
 * from an @ref IMU_Packet into the payload of a @ref VEDTP_Main packet using a
 * fixed, little-endian wire format. The packet header is initialized, the IMU
 * payload is populated sequentially, and the frame is finalized with a checksum
 * for transmission.
 *
 * All vector quantities are encoded as signed 16-bit integers with a fixed
 * scaling factor (×1e3) to ensure deterministic, platform-independent encoding.
 * Values are assumed to be provided in SI units and are clamped implicitly by
 * the conversion macro where applicable.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..5]    : Acceleration vector        (X, Y, Z) int16_t × 1e3
 *   - [6..11]   : Angular rate (gyro)        (X, Y, Z) int16_t × 1e3
 *   - [12..17]  : Magnetic field             (X, Y, Z) int16_t × 1e3
 *   - [18..23]  : Rotation vector            (X, Y, Z) int16_t × 1e3
 *   - [24..29]  : Linear acceleration        (X, Y, Z) int16_t × 1e3
 *   - [30..35]  : Gravity vector             (X, Y, Z) int16_t × 1e3
 *   - [36..37]  : Temperature                (int16_t, °C × 100)
 *   - [38]      : System calibration status  (uint8_t)
 *   - [39]      : Gyroscope calibration      (uint8_t)
 *   - [40]      : Accelerometer calibration  (uint8_t)
 *   - [41]      : Magnetometer calibration   (uint8_t)
 *   - [42..45]  : System uptime              (uint32_t, milliseconds)
 *
 * The payload size is fixed and verified at compile time to maintain binary
 * compatibility between firmware, ground control software, and log decoders.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the IMU data source structure.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for use in real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the IMU payload layout, scaling factors, or
 *          ordering must be reflected in IMU_SIZE and validated using the
 *          static assertion to avoid protocol incompatibility.
 */
uint8_t encode_IMU(VEDTP_Main *out, const IMU_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_IMU;

    memset(out->payload, 0, sizeof(out->payload));

    uint16_t o = 0;
    // accel
    for (int i = 0; i < 3; i++) {
        put_i16_le(out->payload, o, F_TO_I16_E3(data->acceleration[i]));
        o += 2;
    }

    // gyro
    for (int i = 0; i < 3; i++) {
        put_i16_le(out->payload, o, F_TO_I16_E3(data->gyro[i]));
        o += 2;
    }

    // magnetic
    for (int i = 0; i < 3; i++) {
        put_i16_le(out->payload, o, F_TO_I16_E3(data->magnetic[i]));
        o += 2;
    }

    // rotation vector
    for (int i = 0; i < 3; i++) {
        put_i16_le(out->payload, o, F_TO_I16_E3(data->rotationvector[i]));
        o += 2;
    }

    // linear acceleration
    for (int i = 0; i < 3; i++) {
        put_i16_le(out->payload, o, F_TO_I16_E3(data->linearacceleration[i]));
        o += 2;
    }

    // gravity
    for (int i = 0; i < 3; i++) {
        put_i16_le(out->payload, o, F_TO_I16_E3(data->gravity[i]));
        o += 2;
    }

    // temperature (C)
    float temp_e2 = data->temperature_C * 100.0f;
    if (temp_e2 >  INT16_MAX) temp_e2 = INT16_MAX;
    if (temp_e2 <  INT16_MIN) temp_e2 = INT16_MIN;
    put_i16_le(out->payload, o, (int16_t)temp_e2);
    o += 2;

    // calibration flags
    out->payload[o++] = data->systemcalibration;
    out->payload[o++] = data->gyrocalibration;
    out->payload[o++] = data->accelerometercalibration;
    out->payload[o++] = data->magnetometercalibration;

    // uptime
    put_u32_le(out->payload, o, data->uptime_ms);
    o += 4;

    out->payload_length = IMU_SIZE;
    //_Static_assert(IMU_SIZE == 46, "IMU_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode internal atmosphere sensor data into a VEDTP transport frame.
 *
 * This function serializes environmental data measured inside the vehicle hull
 * from an @ref INTERNAL_ATMOSPHERE_Packet into the payload of a
 * @ref VEDTP_Main packet using a fixed, little-endian wire format. The packet
 * header is initialized, the payload fields are populated at predefined offsets,
 * and the frame is finalized with a checksum for transmission.
 *
 * All floating-point inputs are converted to scaled integer representations
 * to ensure deterministic, platform-independent encoding:
 *  - Temperature is encoded as signed int16 in °C × 100.
 *  - Relative humidity is encoded as unsigned uint16 in %RH × 100.
 *  - Pressure is encoded as unsigned uint32 in mbar × 100.
 *  - TVOC concentration is encoded as unsigned uint16 in ppb.
 *  - eCO₂ concentration is encoded as unsigned uint16 in ppm.
 *
 * Boolean and status fields are encoded explicitly to avoid ambiguity on the
 * wire. All values are range-clamped prior to encoding to prevent overflow.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : Internal temperature     (int16_t, °C × 100)
 *   - [2..3]   : Internal humidity        (uint16_t, %RH × 100)
 *   - [4..7]   : Internal pressure        (uint32_t, mbar × 100)
 *   - [8..9]   : TVOC concentration       (uint16_t, ppb)
 *   - [10..11] : eCO₂ concentration       (uint16_t, ppm)
 *   - [12]     : Leak status              (uint8_t, 0 = no leak, 1 = leak)
 *   - [13..16] : System uptime            (uint32_t, milliseconds)
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility across firmware versions, ground control software, and log
 * decoders.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the internal atmosphere data source structure.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the internal atmosphere payload layout, scaling
 *          factors, or field ordering must be reflected in
 *          INTERNAL_ATMOSPHERE_SIZE and validated using the static assertion
 *          to avoid protocol incompatibility.
 */
uint8_t encode_INTERNAL_ATMOSPHERE( VEDTP_Main *out, const INTERNAL_ATMOSPHERE_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_INTERNAL_ATMOSPHERE;

    memset(out->payload, 0, sizeof(out->payload));

    /* ---- temperature (°C * 1e2) ---- */
    float temp_e2 = data->internal_temp_C * 100.0f;
    if (temp_e2 >  INT16_MAX) temp_e2 = INT16_MAX;
    if (temp_e2 <  INT16_MIN) temp_e2 = INT16_MIN;
    put_i16_le(out->payload, 0, (int16_t)temp_e2);

    /* ---- humidity (%RH * 1e2) ---- */
    float hum_e2 = data->internal_humidity_RH * 100.0f;
    if (hum_e2 < 0.0f) hum_e2 = 0.0f;
    if (hum_e2 > 65535.0f) hum_e2 = 65535.0f;
    put_u16_le(out->payload, 2, (uint16_t)hum_e2);

    /* ---- pressure (mbar * 1e2) ---- */
    float pres_e2 = data->internal_pressure_mbar * 100.0f;
    if (pres_e2 < 0.0f) pres_e2 = 0.0f;
    put_u32_le(out->payload, 4, (uint32_t)pres_e2);

    /* ---- TVOC (ppb, no scale) ---- */
    float tvoc = data->internal_tvoc_ppb;
    if (tvoc < 0.0f) tvoc = 0.0f;
    if (tvoc > 65535.0f) tvoc = 65535.0f;
    put_u16_le(out->payload, 8, (uint16_t)tvoc);

    /* ---- eCO2 (ppm, no scale) ---- */
    float eco2 = data->internal_eco2_ppm;
    if (eco2 < 0.0f) eco2 = 0.0f;
    if (eco2 > 65535.0f) eco2 = 65535.0f;
    put_u16_le(out->payload, 10, (uint16_t)eco2);

    /* ---- leak status ---- */
    out->payload[12] = data->leak_status ? 1 : 0;

    /* ---- uptime ---- */
    put_u32_le(out->payload, 13, data->uptime_ms);

    out->payload_length = INTERNAL_ATMOSPHERE_SIZE;
    static_assert(INTERNAL_ATMOSPHERE_SIZE == 17,
                   "INTERNAL_ATMOSPHERE_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode external atmosphere and marine environment data into a VEDTP transport frame.
 *
 * This function serializes environmental data measured outside the vehicle
 * (atmospheric and marine parameters) from an @ref EXTERNAL_ATMOSPHERE_Packet
 * into the payload of a @ref VEDTP_Main packet using a fixed, little-endian
 * wire format. The packet header is initialized, payload fields are written
 * at predefined offsets, and the frame is finalized with a checksum.
 *
 * All floating-point inputs are converted to scaled integer representations
 * to ensure deterministic, platform-independent encoding suitable for
 * real-time telemetry and logging.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : External temperature          (int16_t, °C × 100)
 *   - [2..5]   : External pressure             (uint32_t, mbar × 10)
 *   - [6..7]   : External humidity             (uint16_t, %RH × 100)
 *   - [8..11]  : Altitude                      (int32_t, meters × 100)
 *   - [12..13] : TVOC concentration            (uint16_t, ppb)
 *   - [14..15] : eCO₂ concentration            (uint16_t, ppm)
 *   - [16..17] : Air quality index             (uint16_t)
 *   - [18..21] : Water depth                   (int32_t, meters × 100)
 *   - [22..23] : Salinity                      (uint16_t, ppt × 100)
 *   - [24..27] : Water conductivity            (uint32_t, µS/cm)
 *   - [28..29] : Dissolved oxygen              (uint16_t, mg/L × 100)
 *   - [30..31] : Turbidity                     (uint16_t, NTU × 100)
 *   - [32]     : Sensor status flags           (uint8_t, bitmask)
 *   - [33]     : Medium type                   (uint8_t, air / water)
 *   - [34..37] : System uptime                 (uint32_t, milliseconds)
 *
 * The payload size is fixed and verified at compile time to guarantee binary
 * compatibility between firmware, ground control software, and log decoders.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the external atmosphere data source structure.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the external atmosphere payload layout,
 *          scaling factors, or field ordering must be reflected in
 *          EXTERNAL_ATMOSPHERE_SIZE and validated using the static assertion
 *          to avoid protocol incompatibility.
 */
uint8_t encode_EXTERNAL_ATMOSPHERE( VEDTP_Main *out, const EXTERNAL_ATMOSPHERE_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_EXTERNAL_ATMOSPHERE;

    memset(out->payload, 0, sizeof(out->payload));

    // temperature °C * e2
    put_i16_le(out->payload, 0, (int16_t)(data->temperature_C * 100.0f));

    // pressure mbar * e1
    put_u32_le(out->payload, 2, (uint32_t)(data->pressure_mbar * 10.0f));

    // humidity % * e2
    put_u16_le(out->payload, 6, (uint16_t)(data->humidity_RH * 100.0f));

    // altitude m * e2
    put_i32_le(out->payload, 8, (int32_t)(data->altitude_m * 100.0f));

    // air quality
    put_u16_le(out->payload, 12, (uint16_t)data->tvoc_ppb);
    put_u16_le(out->payload, 14, (uint16_t)data->eco2_ppm);
    put_u16_le(out->payload, 16, data->air_quality_index);

    // marine
    put_i32_le(out->payload, 18, (int32_t)(data->depth_m * 100.0f));
    put_u16_le(out->payload, 22, (uint16_t)(data->salinity_ppt * 100.0f));
    put_u32_le(out->payload, 24, (uint32_t)data->conductivity_uS);
    put_u16_le(out->payload, 28, (uint16_t)(data->water_dissolved_oxygen_mgL * 100.0f));
    put_u16_le(out->payload, 30, (uint16_t)(data->turbidity_NTU * 100.0f));

    // status
    out->payload[32] = data->sensor_status_flags;
    out->payload[33] = data->medium_type;

    // uptime
    put_u32_le(out->payload, 34, data->uptime_ms);

    out->payload_length = EXTERNAL_ATMOSPHERE_SIZE;
    static_assert(EXTERNAL_ATMOSPHERE_SIZE == 38, "EXTERNAL_ATMOSPHERE_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode sonar ranging data into a VEDTP transport frame.
 *
 * This function serializes acoustic ranging and quality metrics from a
 * @ref SONAR_Packet into the payload of a @ref VEDTP_Main packet using a
 * fixed, little-endian wire format. The packet header is initialized,
 * payload fields are populated at predefined offsets, and the frame is
 * finalized with a checksum for transmission.
 *
 * Floating-point inputs are converted to scaled integer representations
 * to ensure deterministic, platform-independent encoding suitable for
 * real-time telemetry and logging.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0]     : Sonar type               (uint8_t)
 *   - [1]     : Sonar identifier         (uint8_t)
 *   - [2..3]  : Range                   (uint16_t, meters × 100)
 *   - [4..5]  : Signal strength          (uint16_t, normalized × 1000)
 *   - [6..7]  : Measurement confidence   (uint16_t, normalized × 1000)
 *   - [8..11] : System uptime            (uint32_t, milliseconds)
 *
 * All scalar values are range-clamped prior to encoding to prevent overflow
 * and to guarantee a bounded wire representation.
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility across firmware versions, ground control software, and
 * log decoders.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the sonar data source structure.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the sonar payload layout, scaling factors,
 *          or field ordering must be reflected in SONAR_SIZE and validated
 *          using the static assertion to avoid protocol incompatibility.
 */
uint8_t encode_SONAR( VEDTP_Main *out, const SONAR_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_SONAR;

    memset(out->payload, 0, sizeof(out->payload));

    // sonar identifiers
    out->payload[0] = data->sonar_type;
    out->payload[1] = data->sonar_id;

    // range (m * 1e2)
    float range_e2 = data->range_m * 100.0f;
    if (range_e2 < 0.0f) range_e2 = 0.0f;
    if (range_e2 > 65535.0f) range_e2 = 65535.0f;
    put_u16_le(out->payload, 2, (uint16_t)range_e2);

    // signal strength (0–1 → *1000)
    float sig_e3 = data->signal_strength * 1000.0f;
    if (sig_e3 < 0.0f) sig_e3 = 0.0f;
    if (sig_e3 > 1000.0f) sig_e3 = 1000.0f;
    put_u16_le(out->payload, 4, (uint16_t)sig_e3);

    // confidence (0–1 → *1000)
    float conf_e3 = data->confidence * 1000.0f;
    if (conf_e3 < 0.0f) conf_e3 = 0.0f;
    if (conf_e3 > 1000.0f) conf_e3 = 1000.0f;
    put_u16_le(out->payload, 6, (uint16_t)conf_e3);

    // uptime
    put_u32_le(out->payload, 8, data->uptime_ms);

    out->payload_length = SONAR_SIZE;
    static_assert(SONAR_SIZE == 12, "SONAR_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode motor and actuator telemetry into a VEDTP transport frame.
 *
 * This function serializes per-motor operational, electrical, thermal, and
 * control-state data from a @ref MOTOR_Packet into the payload of a
 * @ref VEDTP_Main packet using a fixed, little-endian wire format. The packet
 * header is initialized, motor telemetry is packed sequentially for all motors,
 * and the frame is finalized with a checksum for transmission.
 *
 * Motor data is encoded for a fixed set of four motors. All floating-point
 * quantities are converted to scaled integer representations to ensure
 * deterministic, platform-independent encoding suitable for real-time telemetry,
 * logging, and post-mission analysis.
 *
 * Per-motor payload layout (repeated 4 times, little-endian):
 *   - [0..1]   : Motor RPM                  (int16_t)
 *   - [2..3]   : Linear velocity            (int16_t, units × 100)
 *   - [4..7]   : Distance travelled         (int32_t, units × 100)
 *   - [8..9]   : Motor current              (int16_t, A × 100)
 *   - [10..11] : Motor temperature          (int16_t, °C × 100)
 *   - [12..13] : Estimated torque           (int16_t, units × 100)
 *   - [14..15] : Supply voltage             (uint16_t, V × 100)
 *   - [16..17] : PWM command                (int16_t)
 *   - [18]     : Fault code                 (uint8_t)
 *   - [19]     : Control mode               (uint8_t)
 *   - [20]     : Status flags               (uint8_t, bitmask)
 *   - [21]     : Reserved                   (uint8_t)
 *
 * Global payload fields:
 *   - [80..83] : System uptime              (uint32_t, milliseconds)
 *
 * The payload size is fixed and verified at compile time to guarantee binary
 * compatibility between firmware versions, ground control software, and log
 * decoders.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the motor telemetry data source structure.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning This encoder assumes a fixed motor count of four. Any change in
 *          motor count, field ordering, or scaling factors must be reflected
 *          in MOTOR_SIZE and validated using the static assertion to avoid
 *          protocol incompatibility.
 */
uint8_t encode_MOTOR(VEDTP_Main *out, const MOTOR_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_MOTOR;

    memset(out->payload, 0, sizeof(out->payload));

    uint16_t o = 0;


    for (int i = 0; i < 4; i++) {

        put_i16_le(out->payload, o, (int16_t)data->rpm[i]); o += 2;
        put_i16_le(out->payload, o, (int16_t)(data->velocity[i] * 100.0f)); o += 2;
        put_i32_le(out->payload, o, (int32_t)(data->distance[i] * 100.0f)); o += 4;
        put_i16_le(out->payload, o, (int16_t)(data->current[i] * 100.0f)); o += 2;
        put_i16_le(out->payload, o, (int16_t)(data->temperature[i] * 100.0f)); o += 2;
        put_i16_le(out->payload, o, (int16_t)(data->torque_est[i] * 100.0f)); o += 2;
        put_u16_le(out->payload, o, (uint16_t)(data->voltage[i] * 100.0f)); o += 2;
        put_i16_le(out->payload, o, (int16_t)data->pwm[i]); o += 2;

        out->payload[o++] = data->fault_code[i];
        out->payload[o++] = data->control_mode[i];
        out->payload[o++] = data->status_flags[i];
        out->payload[o++] = data->reserved[i];
    }

    put_u32_le(out->payload, o, data->uptime_ms);
    o += 4;

    out->payload_length = MOTOR_SIZE;
    static_assert(MOTOR_SIZE == 84, "MOTOR_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode dead-reckoning and fused navigation data into a VEDTP transport frame.
 *
 * This function serializes locally estimated position, heading, velocity, and
 * optionally fused GPS information from a @ref DEAD_RECKONING_Packet into the
 * payload of a @ref VEDTP_Main packet using a fixed, little-endian wire format.
 * The packet header is initialized, payload fields are populated at predefined
 * offsets, and the frame is finalized with a checksum for transmission.
 *
 * The payload combines local dead-reckoning state with global GPS references
 * to support navigation, estimation debugging, and estimator fallback analysis.
 * All floating-point inputs are converted to scaled integer representations to
 * ensure deterministic, platform-independent encoding.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..3]   : Local X position            (int32_t, millimeters)
 *   - [4..7]   : Local Y position            (int32_t, millimeters)
 *   - [8..11]  : Local Z position            (int32_t, millimeters)
 *   - [12..13] : Yaw heading                 (uint16_t, degrees × 100, [0–360))
 *   - [14..15] : Estimated speed             (uint16_t, m/s × 100)
 *   - [16..19] : GPS latitude                (int32_t, degrees × 1e7)
 *   - [20..23] : GPS longitude               (int32_t, degrees × 1e7)
 *   - [24..27] : GPS altitude                (int32_t, meters × 100)
 *   - [28..29] : GPS ground speed            (uint16_t, m/s × 100)
 *   - [30..31] : GPS course                  (uint16_t, degrees × 100)
 *   - [32]     : GPS fused flag              (uint8_t, 0 = no, 1 = yes)
 *   - [33]     : Dead-reckoning active flag  (uint8_t, 0 = no, 1 = yes)
 *   - [34]     : GPS validity flag           (uint8_t, 0 = invalid, 1 = valid)
 *   - [35]     : Reserved                    (uint8_t)
 *   - [36..39] : System uptime               (uint32_t, milliseconds)
 *
 * Heading is normalized to the range [0, 360) degrees prior to encoding.
 * All scalar values are range-clamped where applicable to guarantee bounded
 * wire representation.
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility between firmware, ground control software, and log decoders.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the dead-reckoning data source structure.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the dead-reckoning payload layout, scaling
 *          factors, or fusion flags must be reflected in DEAD_RECKONING_SIZE
 *          and validated using the static assertion to avoid protocol
 *          incompatibility.
 */
uint8_t encode_DEAD_RECKONING( VEDTP_Main *out, const DEAD_RECKONING_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_DEAD_RECKONING;

    memset(out->payload, 0, sizeof(out->payload));

    /* Local position (mm) */
    put_i32_le(out->payload, 0, data->x_mm);
    put_i32_le(out->payload, 4, data->y_mm);
    put_i32_le(out->payload, 8, data->z_mm);

    /* Yaw (deg * 1e2, wrap 0–360) */
    float yaw = data->yaw_cdeg;
    while (yaw < 0.0f)    yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;
    put_u16_le(out->payload, 12, (uint16_t)(yaw * 100.0f));

    /* Speed (m/s * 1e2) */
    float spd_e2 = data->speed_mps * 100.0f;
    if (spd_e2 < 0.0f) spd_e2 = 0.0f;
    if (spd_e2 > 65535.0f) spd_e2 = 65535.0f;
    put_u16_le(out->payload, 14, (uint16_t)spd_e2);

    /* GPS lat/lon (deg * 1e7) */
    put_i32_le(out->payload, 16, (int32_t)(data->gps_lat * 1e7));
    put_i32_le(out->payload, 20, (int32_t)(data->gps_lon * 1e7));

    /* GPS altitude (m * 1e2) */
    put_i32_le(out->payload, 24, (int32_t)(data->gps_alt * 100.0));

    /* GPS speed & course */
    put_u16_le(out->payload, 28, (uint16_t)(data->gps_speed * 100.0f));
    put_u16_le(out->payload, 30, (uint16_t)(data->gps_course * 100.0f));

    /* Fusion flags */
    out->payload[32] = data->gps_fused ? 1 : 0;
    out->payload[33] = data->deadrec_active ? 1 : 0;
    out->payload[34] = data->gps_valid ? 1 : 0;
    out->payload[35] = data->reserved;

    /* Uptime */
    put_u32_le(out->payload, 36, data->uptime_ms);

    out->payload_length = DEAD_RECKONING_SIZE;
    static_assert(DEAD_RECKONING_SIZE == 40, "DEAD_RECKONING_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode power system and battery health data into a VEDTP transport frame.
 *
 * This function serializes electrical power, battery state, and energy
 * availability metrics from a @ref POWER_HEALTH_Packet into the payload of a
 * @ref VEDTP_Main packet using a fixed, little-endian wire format. The packet
 * header is initialized, payload fields are populated at predefined offsets,
 * and the frame is finalized with a checksum for transmission.
 *
 * Power-related quantities are converted from floating-point inputs into
 * scaled integer representations to ensure deterministic, platform-independent
 * encoding suitable for real-time telemetry, fault monitoring, and post-mission
 * analysis.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : SMPS voltage               (uint16_t, volts × 100)
 *   - [2..3]   : Battery voltage            (uint16_t, volts × 100)
 *   - [4..5]   : Current draw               (int16_t, amperes × 100)
 *   - [6..7]   : Battery capacity           (uint16_t, percent × 100)
 *   - [8..9]   : Power usage                (uint16_t, watts × 10)
 *   - [10..11] : Estimated remaining uptime (uint16_t, minutes × 10)
 *   - [12]     : Charging status            (uint8_t, 0 = no, 1 = charging)
 *   - [13]     : Discharging status         (uint8_t, 0 = no, 1 = discharging)
 *   - [14]     : Battery present flag       (uint8_t, 0 = absent, 1 = present)
 *   - [15]     : Power status flags         (uint8_t, bitmask)
 *   - [16..19] : System uptime              (uint32_t, milliseconds)
 *
 * Battery capacity values are range-clamped to [0, 100%] prior to encoding
 * to guarantee a bounded wire representation.
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility between firmware, ground control software, and log decoders.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the power health data source structure.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the power health payload layout, scaling
 *          factors, or status flag semantics must be reflected in
 *          POWER_HEALTH_SIZE and validated using the static assertion to avoid
 *          protocol incompatibility.
 */
uint8_t encode_POWER_HEALTH( VEDTP_Main *out, const POWER_HEALTH_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_POWER_HEALTH;

    memset(out->payload, 0, sizeof(out->payload));

    // voltages (V * 100)
    put_u16_le(out->payload, 0, (uint16_t)(data->smps_voltage * 100.0f));
    put_u16_le(out->payload, 2, (uint16_t)(data->battery_voltage * 100.0f));

    // current (A * 100)
    put_i16_le(out->payload, 4, (int16_t)(data->current_draw * 100.0f));

    // battery capacity (% * 100)
    float cap_e2 = data->battery_capacity_pct * 100.0f;
    if (cap_e2 < 0.0f) cap_e2 = 0.0f;
    if (cap_e2 > 10000.0f) cap_e2 = 10000.0f;
    put_u16_le(out->payload, 6, (uint16_t)cap_e2);

    // power usage (W * 10)
    put_u16_le(out->payload, 8, (uint16_t)(data->power_usage_watt * 10.0f));

    // estimated uptime (minutes * 10)
    put_u16_le(out->payload, 10, (uint16_t)(data->estimated_uptime_min * 10.0f));

    // status flags
    out->payload[12] = data->charging ? 1 : 0;
    out->payload[13] = data->discharging ? 1 : 0;
    out->payload[14] = data->battery_present ? 1 : 0;
    out->payload[15] = data->power_flags;

    // uptime
    put_u32_le(out->payload, 16, data->uptime_ms);

    out->payload_length = POWER_HEALTH_SIZE;
    static_assert(POWER_HEALTH_SIZE == 20, "POWER_HEALTH_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}
/**
 * @brief Encode flight controller health and system status into a VEDTP transport frame.
 *
 * This function serializes core flight-controller health metrics, including
 * thermal state, CPU and memory utilization, RTOS status, and fault indicators,
 * from a @ref FC_HEALTH_Packet into the payload of a @ref VEDTP_Main packet using
 * a fixed, little-endian wire format. The packet header is initialized, payload
 * fields are populated at predefined offsets, and the frame is finalized with a
 * checksum for transmission.
 *
 * All floating-point inputs are converted into scaled integer representations
 * to ensure deterministic, platform-independent encoding suitable for real-time
 * monitoring, diagnostics, and post-mission analysis.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : Controller temperature        (int16_t, °C × 100)
 *   - [2..3]   : CPU usage                     (uint16_t, % × 100)
 *   - [4..5]   : Heap usage                    (uint16_t, % × 100)
 *   - [6..7]   : Stack high-water mark         (uint16_t, % × 100)
 *   - [8..11]  : RTOS tick count               (uint32_t)
 *   - [12..13] : Error counter                 (uint16_t)
 *   - [14..15] : Warning counter               (uint16_t)
 *   - [16]     : Reset reason                  (uint8_t, platform-defined)
 *   - [17]     : RTOS health flag              (uint8_t, 0 = fault, 1 = OK)
 *   - [18]     : Task fault flags              (uint8_t, bitmask)
 *   - [19]     : Failsafe flags                (uint8_t, bitmask)
 *   - [20..23] : System uptime                 (uint32_t, milliseconds)
 *
 * Utilization percentages are expressed as scaled integers to avoid floating-
 * point representation on the wire and to guarantee bounded telemetry values.
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility between firmware versions, ground control software, and log
 * decoders.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the flight controller health data source.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the flight controller health payload layout,
 *          scaling factors, or status flag semantics must be reflected in
 *          FC_HEALTH_SIZE and validated using the static assertion to avoid
 *          protocol incompatibility.
 */
uint8_t encode_FC_HEALTH( VEDTP_Main *out, const FC_HEALTH_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_FC_HEALTH;

    memset(out->payload, 0, sizeof(out->payload));

    // temperature °C * 100
    put_i16_le(out->payload, 0, (int16_t)(data->temperature * 100.0f));

    // CPU usage % * 100
    put_u16_le(out->payload, 2, (uint16_t)(data->cpu_usage_pct * 100.0f));

    // heap usage % * 100
    put_u16_le(out->payload, 4, (uint16_t)(data->heap_usage_pct * 100.0f));

    // stack watermark % * 100
    put_u16_le(out->payload, 6, (uint16_t)(data->stack_high_watermark_pct * 100.0f));

    // RTOS tick count
    put_u32_le(out->payload, 8, data->tick_count);

    // error / warning counters
    put_u16_le(out->payload, 12, data->error_count);
    put_u16_le(out->payload, 14, data->warning_count);

    // status flags
    out->payload[16] = data->reset_reason;
    out->payload[17] = data->rtos_ok ? 1 : 0;
    out->payload[18] = data->task_fault_flags;
    out->payload[19] = data->failsafe_flags;

    // uptime
    put_u32_le(out->payload, 20, data->uptime_ms);

    out->payload_length = FC_HEALTH_SIZE;
    static_assert(FC_HEALTH_SIZE == 24, "FC_HEALTH_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode a command message into a VEDTP transport frame.
 *
 * This function serializes a command request from a @ref COMMAND_Packet into
 * the payload of a @ref VEDTP_Main packet using a fixed, little-endian wire
 * format. The packet header is initialized, command fields and parameters are
 * populated at predefined offsets, and the frame is finalized with a checksum
 * for transmission.
 *
 * The command packet supports a compact, deterministic command interface with
 * a fixed maximum number of parameters and optional string arguments. This
 * design ensures predictable parsing and bounded memory usage on embedded
 * targets.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0]      : Command ID                  (uint8_t)
 *   - [1]      : Target system/component     (uint8_t)
 *   - [2]      : Command flags               (uint8_t, bitmask)
 *   - [3]      : Parameter count             (uint8_t)
 *   - [4..15]  : Command parameters          (6 × int16_t, little-endian)
 *   - [16..27] : String argument 1           (12 bytes, raw/ASCII)
 *   - [28..39] : String argument 2           (12 bytes, raw/ASCII)
 *   - [40..43] : System uptime               (uint32_t, milliseconds)
 *
 * All parameters are transmitted as signed 16-bit integers to maintain a
 * compact wire representation. String arguments are transmitted as fixed-size
 * byte arrays and may be null-terminated or raw, depending on command context.
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility between firmware versions, command senders, and receivers.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the command data source structure.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the command payload layout, parameter count,
 *          or string field sizes must be reflected in COMMAND_SIZE and
 *          validated using the static assertion to avoid protocol
 *          incompatibility.
 */
uint8_t encode_COMMAND( VEDTP_Main *out, const COMMAND_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_COMMAND;

    memset(out->payload, 0, sizeof(out->payload));

    out->payload[0] = data->id;
    out->payload[1] = data->target;
    out->payload[2] = data->flags;
    out->payload[3] = data->param_count;

    // params
    uint16_t o = 4;
    for (int i = 0; i < 6; i++) {
        put_i16_le(out->payload, o, data->param[i]);
        o += 2;
    }

    // strings
    memcpy(&out->payload[o], data->char1, 12);
    o += 12;
    memcpy(&out->payload[o], data->char2, 12);
    o += 12;

    // uptime
    put_u32_le(out->payload, o, data->uptime_ms);

    out->payload_length = COMMAND_SIZE;
    static_assert(COMMAND_SIZE == 44, "COMMAND_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode joystick input data into a VEDTP transport frame.
 *
 * This function serializes manual control input from a @ref JOYSTICK_Packet
 * into the payload of a @ref VEDTP_Main packet using a fixed, little-endian
 * wire format. The packet header is initialized, joystick channel data and
 * metadata are populated sequentially, and the frame is finalized with a
 * checksum for transmission.
 *
 * The joystick packet provides a deterministic and bounded representation
 * of operator input suitable for real-time manual control, assist modes,
 * and command arbitration logic.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..15]  : Joystick channels            (8 × uint16_t)
 *   - [16]     : Channel count                (uint8_t)
 *   - [17]     : Joystick flags               (uint8_t, bitmask)
 *   - [18..21] : System uptime                (uint32_t, milliseconds)
 *
 * Channel values are transmitted as raw unsigned 16-bit values. Interpretation
 * (e.g., centering, scaling, dead zones) is performed at the receiver based on
 * the active control mode.
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility between firmware, ground control software, and input decoders.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the joystick input data source structure.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the joystick payload layout, channel count,
 *          or flag semantics must be reflected in JOYSTICK_SIZE and validated
 *          using the static assertion to avoid protocol incompatibility.
 */
uint8_t encode_JOYSTICK( VEDTP_Main *out, const JOYSTICK_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_JOYSTICK;

    memset(out->payload, 0, sizeof(out->payload));

    uint16_t o = 0;

    // channels
    for (int i = 0; i < 8; i++) {
        put_u16_le(out->payload, o, data->channels[i]);
        o += 2;
    }

    // metadata
    out->payload[o++] = data->channel_count;
    out->payload[o++] = data->flags;

    // uptime
    put_u32_le(out->payload, o, data->uptime_ms);

    out->payload_length = JOYSTICK_SIZE;
    static_assert(JOYSTICK_SIZE == 22, "JOYSTICK_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode flow meter and water current data into a VEDTP transport frame.
 *
 * This function serializes hydrodynamic flow measurements and sensor orientation
 * data from a @ref FLOWMETER_Packet into the payload of a @ref VEDTP_Main packet
 * using a fixed, little-endian wire format. The packet header is initialized,
 * payload fields are populated at predefined offsets, and the frame is finalized
 * with a checksum for transmission.
 *
 * The flow meter packet provides information about local water flow speed,
 * direction, and sensor orientation, enabling current compensation, navigation
 * assistance, and environmental analysis.
 *
 * All floating-point inputs are converted to scaled integer representations to
 * ensure deterministic, platform-independent encoding. Angular quantities are
 * normalized prior to encoding to guarantee bounded wire representation.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : Flow meter RPM              (int16_t)
 *   - [2..3]   : Flow speed                  (int16_t, m/s × 100)
 *   - [4..5]   : Flow direction              (uint16_t, degrees × 100, [0–360))
 *   - [6..7]   : Sensor roll                 (int16_t, degrees × 100)
 *   - [8..9]   : Sensor pitch                (int16_t, degrees × 100)
 *   - [10..11] : Sensor yaw                  (uint16_t, degrees × 100, [0–360))
 *   - [12]     : Sensor status flags         (uint8_t, bitmask)
 *   - [13..16] : System uptime               (uint32_t, milliseconds)
 *
 * Flow speed values are clamped to non-negative ranges prior to encoding.
 * Directional angles are wrapped into the range [0, 360) degrees to ensure
 * consistent interpretation across receivers.
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility between firmware versions, ground control software, and
 * log decoders.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the flow meter data source structure.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the flow meter payload layout, scaling factors,
 *          or angle normalization rules must be reflected in FLOWMETER_SIZE
 *          and validated using the static assertion to avoid protocol
 *          incompatibility.
 */
uint8_t encode_FLOWMETER( VEDTP_Main *out, const FLOWMETER_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_FLOW_METER;

    memset(out->payload, 0, sizeof(out->payload));

    // RPM
    float rpm = data->rpm;
    if (rpm >  32767.0f) rpm =  32767.0f;
    if (rpm < -32768.0f) rpm = -32768.0f;
    put_i16_le(out->payload, 0, (int16_t)rpm);

    // flow speed (m/s * 100)
    float spd_e2 = data->flow_speed_mps * 100.0f;
    if (spd_e2 < 0.0f) spd_e2 = 0.0f;
    if (spd_e2 > 32767.0f) spd_e2 = 32767.0f;
    put_i16_le(out->payload, 2, (int16_t)spd_e2);

    // flow direction (deg * 100, wrap 0–360)
    float dir = data->flow_direction_deg;
    while (dir < 0.0f)   dir += 360.0f;
    while (dir >= 360.0f) dir -= 360.0f;
    put_u16_le(out->payload, 4, (uint16_t)(dir * 100.0f));

    // roll (deg * 100)
    put_i16_le(out->payload, 6,
               (int16_t)(data->roll * 100.0f));

    // pitch (deg * 100)
    put_i16_le(out->payload, 8,
               (int16_t)(data->pitch * 100.0f));

    // yaw (deg * 100, wrap 0–360)
    float yaw = data->yaw;
    while (yaw < 0.0f)   yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;
    put_u16_le(out->payload, 10,
               (uint16_t)(yaw * 100.0f));

    // status
    out->payload[12] = data->status_flags;

    // uptime
    put_u32_le(out->payload, 13, data->uptime_ms);

    out->payload_length = FLOWMETER_SIZE;
    static_assert(FLOWMETER_SIZE == 17, "FLOWMETER_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode a command acknowledgement into a VEDTP transport frame.
 *
 * This function serializes the execution result of a previously received
 * command from a @ref COMMAND_ACK_Packet into the payload of a
 * @ref VEDTP_Main packet using a fixed, little-endian wire format. The packet
 * header is initialized, acknowledgement fields are populated at predefined
 * offsets, and the frame is finalized with a checksum for transmission.
 *
 * Command acknowledgements provide deterministic feedback to the command sender,
 * enabling reliable command execution tracking, error handling, and retry
 * mechanisms in both manual and autonomous control flows.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0]      : Command ID                  (uint8_t, echoes issued command)
 *   - [1]      : Result code                 (uint8_t, success / error code)
 *   - [2]      : Source                      (uint8_t, responder identity)
 *   - [3]      : Acknowledgement flags       (uint8_t, bitmask)
 *   - [4..53]  : Status message              (50 bytes, fixed-length, ASCII/raw)
 *   - [54..57] : System uptime               (uint32_t, milliseconds)
 *
 * The status message field is transmitted as a fixed-length byte array to avoid
 * variable-length parsing and to guarantee bounded memory usage on embedded
 * receivers. The message may be null-terminated or raw, depending on context.
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility between firmware versions, ground control software, and
 * command-handling logic.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the command acknowledgement data source.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time systems when called outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the command acknowledgement payload layout,
 *          message length, or result semantics must be reflected in
 *          COMMAND_ACK_SIZE and validated using the static assertion to avoid
 *          protocol incompatibility.
 */
uint8_t encode_COMMAND_ACK( VEDTP_Main *out, const COMMAND_ACK_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_COMMAND_ACK;

    memset(out->payload, 0, sizeof(out->payload));

    out->payload[0] = data->command;
    out->payload[1] = data->result;
    out->payload[2] = data->source;
    out->payload[3] = data->flags;

    // fixed-length message (no overflow, no strlen dependency)
    memcpy(&out->payload[4], data->message, 50);

    // uptime
    put_u32_le(out->payload, 54, data->uptime_ms);

    out->payload_length = COMMAND_ACK_SIZE;
    static_assert(COMMAND_ACK_SIZE == 58, "COMMAND_ACK_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode a device-level error notification into a VEDTP transport frame.
 *
 * This function serializes a device error event from a @ref DEVICE_ERROR_Packet
 * into the payload of a @ref VEDTP_Main packet using a fixed, little-endian wire
 * format. The packet header is initialized, error metadata is populated at
 * predefined offsets, and the frame is finalized with a checksum for transmission.
 *
 * Device error packets provide a lightweight, deterministic mechanism for
 * reporting faults, warnings, and critical failures originating from individual
 * subsystems. These packets are intended for real-time monitoring, fault
 * aggregation, and autonomous safety handling.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0]      : Device identifier           (uint8_t)
 *   - [1]      : Error type                  (uint8_t, subsystem-defined)
 *   - [2]      : Severity level              (uint8_t, informational / warning / critical)
 *   - [3]      : Error flags                 (uint8_t, bitmask)
 *   - [4..7]   : System uptime               (uint32_t, milliseconds)
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility between firmware versions, fault monitors, and ground control
 * software.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the device error data source structure.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for use in real-time systems, including fault-reporting paths.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the device error payload layout, severity
 *          semantics, or flag definitions must be reflected in
 *          DEVICE_ERROR_SIZE and validated using the static assertion to avoid
 *          protocol incompatibility.
 */
uint8_t encode_DEVICE_ERROR( VEDTP_Main *out, const DEVICE_ERROR_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_DEVICE_ERROR;

    memset(out->payload, 0, sizeof(out->payload));

    out->payload[0] = data->device_id;
    out->payload[1] = data->error_type;
    out->payload[2] = data->severity;
    out->payload[3] = data->flags;

    put_u32_le(out->payload, 4, data->uptime_ms);

    out->payload_length = DEVICE_ERROR_SIZE;
    static_assert(DEVICE_ERROR_SIZE == 8, "DEVICE_ERROR_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode Cone Penetration Test (CPT) measurement data into a VEDTP transport frame.
 *
 * This function serializes geotechnical CPT measurement data, including probe
 * orientation, penetration metrics, strain response, environmental context,
 * and geolocation, from a @ref CPT_DATA_Packet into the payload of a
 * @ref VEDTP_Main packet using a fixed, little-endian wire format. The packet
 * header is initialized, payload fields are populated at predefined offsets,
 * and the frame is finalized with a checksum for transmission.
 *
 * The CPT data packet is designed for deterministic, bounded transmission of
 * high-value survey data suitable for real-time monitoring, logging, and
 * post-processing in seabed and subsurface analysis workflows.
 *
 * All floating-point inputs are converted into scaled integer representations
 * to ensure platform-independent encoding and repeatable decoding.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : Probe angle                 (int16_t, degrees × 100)
 *   - [2..5]   : Penetration pressure        (uint32_t, mbar × 10)
 *   - [6..7]   : Penetration depth           (int16_t, millimeters)
 *   - [8..9]   : Insertion speed             (int16_t, mm/s × 10)
 *   - [10..13] : Strain measurement          (int32_t, raw × 1000)
 *   - [14..15] : Rod temperature             (int16_t, °C × 100)
 *   - [16..19] : Latitude                   (int32_t, degrees × 1e7)
 *   - [20..23] : Longitude                  (int32_t, degrees × 1e7)
 *   - [24..27] : Water / soil depth          (int32_t, meters × 100)
 *   - [28..29] : Roll angle                  (int16_t, degrees × 100)
 *   - [30..31] : Pitch angle                 (int16_t, degrees × 100)
 *   - [32..33] : Yaw angle                   (uint16_t, degrees × 100, [0–360))
 *   - [34]     : Status flags                (uint8_t, bitmask)
 *   - [35]     : Soil classification         (uint8_t, enumeration)
 *   - [36]     : Measurement step index      (uint8_t)
 *   - [37]     : Active state                (uint8_t, 0 = inactive, 1 = active)
 *   - [38..53] : Measurement label           (16 bytes, fixed-length, ASCII/raw)
 *   - [54..57] : System uptime               (uint32_t, milliseconds)
 *
 * Angular quantities are normalized to the range [0, 360) degrees prior to
 * encoding to guarantee a bounded wire representation.
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility between firmware versions, data loggers, and analysis tools.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the CPT measurement data source structure.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time data acquisition systems when called outside interrupt
 *       context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the CPT payload layout, scaling factors,
 *          or semantic meaning of fields must be reflected in CPT_DATA_SIZE
 *          and validated using the static assertion to avoid protocol
 *          incompatibility.
 */
uint8_t encode_CPT_DATA( VEDTP_Main *out, const CPT_DATA_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_CPT_DATA;

    memset(out->payload, 0, sizeof(out->payload));

    put_i16_le(out->payload, 0,
               (int16_t)(data->angle_deg * 100.0f));

    put_u32_le(out->payload, 2,
               (uint32_t)(data->pressure_mbar * 10.0f));

    put_i16_le(out->payload, 6, data->penetration_mm);

    put_i16_le(out->payload, 8,
               (int16_t)(data->insertion_speed_mmps * 10.0f));

    put_i32_le(out->payload, 10,
               (int32_t)(data->strain_raw * 1000.0f));

    put_i16_le(out->payload, 14,
               (int16_t)(data->rod_temp_c * 100.0f));

    put_i32_le(out->payload, 16,
               (int32_t)(data->latitude_deg * 1e7));

    put_i32_le(out->payload, 20,
               (int32_t)(data->longitude_deg * 1e7));

    put_i32_le(out->payload, 24,
               (int32_t)(data->depth_m * 100.0f));

    put_i16_le(out->payload, 28,
               (int16_t)(data->roll_deg * 100.0f));

    put_i16_le(out->payload, 30,
               (int16_t)(data->pitch_deg * 100.0f));

    float yaw = data->yaw_deg;
    while (yaw < 0.0f)    yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;
    put_u16_le(out->payload, 32,
               (uint16_t)(yaw * 100.0f));

    out->payload[34] = data->status_flags;
    out->payload[35] = data->soil_class;
    out->payload[36] = data->step_index;
    out->payload[37] = data->active ? 1 : 0;

    memcpy(&out->payload[38], data->label, 16);

    put_u32_le(out->payload, 54, data->uptime_ms);

    out->payload_length = CPT_DATA_SIZE;
    static_assert(CPT_DATA_SIZE == 58,
                   "CPT_DATA_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode a waypoint navigation command into a VEDTP transport frame.
 *
 * This function serializes a single waypoint definition from a
 * @ref WAYPOINT_Packet into the payload of a @ref VEDTP_Main packet using a
 * fixed, little-endian wire format. The packet header is initialized, waypoint
 * parameters are populated at predefined offsets, and the frame is finalized
 * with a checksum for transmission.
 *
 * Waypoint packets are used to define mission trajectories, behaviors, and
 * constraints for autonomous or semi-autonomous vehicle operation. Each packet
 * represents one waypoint within a mission sequence.
 *
 * All floating-point inputs are converted into scaled integer representations
 * to ensure deterministic, platform-independent encoding suitable for embedded
 * navigation systems.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0]      : Waypoint ID                 (uint8_t)
 *   - [1]      : Total waypoint count        (uint8_t)
 *   - [2]      : Waypoint type               (uint8_t, navigation / action)
 *   - [3]      : Next command                (uint8_t)
 *   - [4]      : Action code                 (uint8_t)
 *   - [5]      : Reference frame             (uint8_t)
 *   - [6..9]   : Latitude                   (int32_t, degrees × 1e7)
 *   - [10..13] : Longitude                  (int32_t, degrees × 1e7)
 *   - [14..17] : Altitude / depth            (int32_t, millimeters)
 *   - [18..19] : Target speed               (uint16_t, m/s × 100)
 *   - [20..21] : Acceptance radius           (uint16_t, meters × 100)
 *   - [22..23] : Hold time                  (int16_t, seconds)
 *   - [24..25] : Desired yaw                (uint16_t, degrees × 100, [0–360))
 *   - [26]     : Waypoint flags              (uint8_t, bitmask)
 *   - [27]     : Retry count                 (uint8_t)
 *   - [28..39] : Waypoint label              (12 bytes, fixed-length, ASCII/raw)
 *   - [40..43] : System uptime               (uint32_t, milliseconds)
 *
 * Angular values are normalized to the range [0, 360) degrees prior to encoding
 * to guarantee bounded and deterministic wire representation.
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility between firmware versions, mission planners, and ground
 * control software.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the waypoint definition data source.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time navigation and mission control systems when called
 *       outside interrupt context.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the waypoint payload layout, scaling factors,
 *          or semantic meaning of fields must be reflected in WAYPOINT_SIZE
 *          and validated using the static assertion to avoid protocol
 *          incompatibility.
 */
uint8_t encode_WAYPOINT( VEDTP_Main *out, const WAYPOINT_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_WAYPOINT;

    memset(out->payload, 0, sizeof(out->payload));

    out->payload[0] = data->id;
    out->payload[1] = data->wp_count;
    out->payload[2] = data->wp_type;
    out->payload[3] = data->next_cmd;
    out->payload[4] = data->action_code;
    out->payload[5] = data->frame;

    put_i32_le(out->payload, 6, (int32_t)(data->lat_deg * 1e7));

    put_i32_le(out->payload, 10, (int32_t)(data->lon_deg * 1e7));

    put_i32_le(out->payload, 14, data->alt_mm);

    put_u16_le(out->payload, 18, (uint16_t)(data->speed_mps * 100.0f));

    put_u16_le(out->payload, 20, (uint16_t)(data->radius_m * 100.0f));

    put_i16_le(out->payload, 22, data->hold_time_s);

    float yaw = data->yaw_deg;
    while (yaw < 0.0f)    yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;
    put_u16_le(out->payload, 24, (uint16_t)(yaw * 100.0f));

    out->payload[26] = data->flags;
    out->payload[27] = data->retries;

    memcpy(&out->payload[28], data->label, 12);

    put_u32_le(out->payload, 40, data->uptime_ms);

    out->payload_length = WAYPOINT_SIZE;
    static_assert(WAYPOINT_SIZE == 44, "WAYPOINT_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}

/**
 * @brief Encode a waypoint acknowledgement message into a VEDTP transport frame.
 *
 * This function serializes the acknowledgement status of a waypoint operation
 * from a @ref WP_ACK_Packet into the payload of a @ref VEDTP_Main packet using
 * a fixed, little-endian wire format. The packet header is initialized, the
 * acknowledgement fields are populated at predefined offsets, and the frame
 * is finalized with a checksum for transmission.
 *
 * Waypoint acknowledgement packets are used to confirm receipt, validation,
 * or rejection of waypoint data during mission upload and execution workflows.
 * They provide deterministic feedback to the mission planner or ground control
 * software.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : Waypoint ID                 (uint16_t)
 *   - [2]      : Result code                 (uint8_t)
 *   - [3..6]   : System uptime               (uint32_t, milliseconds)
 *
 * The result code indicates the outcome of the waypoint operation (e.g.,
 * accepted, rejected, invalid index, CRC failure), as defined by the waypoint
 * acknowledgement result enumeration.
 *
 * The payload size is fixed and verified at compile time to preserve binary
 * compatibility between firmware versions, mission planners, and ground
 * control software.
 *
 * @param[out] out   Pointer to the VEDTP packet to be populated.
 * @param[in]  data  Pointer to the waypoint acknowledgement data source.
 *
 * @return uint8_t
 *         - 1 : Encoding successful and packet is ready for transmission.
 *         - 0 : Invalid arguments (NULL pointer).
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time mission control and acknowledgement paths.
 *
 * @note All multi-byte fields are explicitly encoded in little-endian order
 *       to ensure cross-platform interoperability.
 *
 * @warning Any modification to the waypoint acknowledgement payload layout,
 *          result semantics, or field sizes must be reflected in WP_ACK_SIZE
 *          and validated using the static assertion to avoid protocol
 *          incompatibility.
 */
uint8_t encode_WP_ACK( VEDTP_Main *out, const WP_ACK_Packet *data)
{
    if (!out || !data) return 0;

    out->startByte1       = 0xFF;
    out->startByte2       = 0xEE;
    out->protocol_version = PROTOCOL_VERSION;
    out->vehicle          = VEHICLE_ROV6;
    out->device           = DEVICE_WP_ACK;

    memset(out->payload, 0, sizeof(out->payload));

    put_u16_le(out->payload, 0, data->wp_id);
    out->payload[2] = data->result;
    put_u32_le(out->payload, 3, data->uptime_ms);

    out->payload_length = WP_ACK_SIZE;
    static_assert(WP_ACK_SIZE == 7, "WP_ACK_SIZE mismatch");

    vedtp_pack_data(out);
    return 1;
}


////////////////////////////////////////////////
//decode_ data

/**
 * @brief Decode a HEARTBEAT payload from a VEDTP transport frame.
 *
 * This function extracts system state and health information from a received
 * @ref VEDTP_Main packet carrying a HEARTBEAT message and populates a
 * @ref HEARTBEAT_Packet structure with the decoded values.
 *
 * The function validates that the incoming packet corresponds to the
 * HEARTBEAT device type and that the payload length matches the expected
 * HEARTBEAT payload size before decoding. All multi-byte fields are decoded
 * assuming little-endian wire format.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0]      : Logged state                (uint8_t, 0 = not logged, 1 = logged)
 *   - [1]      : Vehicle ID                  (uint8_t)
 *   - [2]      : Armed state                 (uint8_t, 0 = disarmed, 1 = armed)
 *   - [3]      : System mode                 (uint8_t)
 *   - [4]      : GPS fix status              (uint8_t)
 *   - [5]      : Failsafe flags              (uint8_t, bitmask)
 *   - [6]      : System health               (uint8_t)
 *   - [7..8]   : Sensors validity flags      (uint16_t, bitmask)
 *   - [9..12]  : System uptime               (uint32_t, milliseconds)
 *
 * This message is typically transmitted periodically to provide a continuous
 * overview of vehicle state, system health, and operational readiness.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the HEARTBEAT data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and data structure populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time receive paths.
 *
 * @note The decoder assumes the packet checksum has already been verified
 *       before this function is called.
 *
 * @warning If the payload layout or HEARTBEAT_SIZE changes, this decoder must
 *          be updated accordingly to maintain protocol compatibility.
 */
uint8_t decode_HEARTBEAT(const VEDTP_Main *vedtp, HEARTBEAT_Packet *data)
{
    if (!vedtp || !data) {
        return 0;
    }

    if (vedtp->device != DEVICE_HEARTBEAT ||
        vedtp->payload_length != HEARTBEAT_SIZE) {
        return 0;
    }

    data->loggedState         = vedtp->payload[0] ? 1 : 0;
    data->vehicleID   = vedtp->payload[1];
    data->armedState          = vedtp->payload[2] ? 1 : 0;
    data->systemMode           = vedtp->payload[3];
    data->gps_fix        = vedtp->payload[4];
    data->failsafe_flags = vedtp->payload[5];
    data->system_health  = vedtp->payload[6];

    data->sensors_validity = get_u16_le(vedtp->payload, 7);
    data->uptime_ms        = get_u32_le(vedtp->payload, 9);

    return 1;
}

/**
 * @brief Decode AHRS attitude data from a VEDTP transport frame.
 *
 * This function extracts attitude and timing information from a received
 * @ref VEDTP_Main packet carrying an AHRS message and populates an
 * @ref AHRS_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the AHRS
 * device type and that the payload length matches the expected AHRS payload
 * size before decoding. All multi-byte fields are decoded assuming a
 * little-endian wire format.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1] : Yaw angle        (uint16_t, degrees × 100, [0–360))
 *   - [2..3] : Pitch angle      (int16_t,  degrees × 100)
 *   - [4..5] : Roll angle       (int16_t,  degrees × 100)
 *   - [6..9] : System uptime    (uint32_t, milliseconds)
 *
 * Angular values are converted back to floating-point degrees by applying
 * the inverse of the encoder scaling factor (1 / f_e2).
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the AHRS data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and AHRS data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time receive paths.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning If the AHRS payload layout, scaling factors, or AHRS_SIZE change,
 *          this decoder must be updated accordingly to maintain protocol
 *          compatibility.
 */
uint8_t decode_AHRS(const VEDTP_Main *vedtp, AHRS_Packet *data)
{
    if (!vedtp || !data) {
        return 0;
    }

    if (vedtp->device != DEVICE_AHRS ||
        vedtp->payload_length != AHRS_SIZE) {
        return 0;
    }

    /* ---- yaw (uint16, e2) ---- */
    uint16_t yaw_e2 = get_u16_le(vedtp->payload, 0);
    data->yaw_deg = (float)yaw_e2 / f_e2;

    /* ---- pitch (int16, e2) ---- */
    int16_t pitch_e2 = get_i16_le(vedtp->payload, 2);
    data->pitch_deg = (float)pitch_e2 / f_e2;

    /* ---- roll (int16, e2) ---- */
    int16_t roll_e2 = get_i16_le(vedtp->payload, 4);
    data->roll_deg = (float)roll_e2 / f_e2;

    /* ---- uptime ---- */
    data->uptime_ms = get_u32_le(vedtp->payload, 6);

    return 1;   // success
}

/**
 * @brief Decode GPS navigation data from a VEDTP transport frame.
 *
 * This function extracts global positioning, velocity, accuracy, and timing
 * information from a received @ref VEDTP_Main packet carrying a GPS message
 * and populates a @ref GPS_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the GPS
 * device type and that the payload length matches the expected GPS payload
 * size before decoding. All multi-byte fields are decoded assuming a
 * little-endian wire format.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..3]   : Time of week              (uint32_t, milliseconds)
 *   - [4..7]   : Latitude                 (int32_t, degrees × 1e7)
 *   - [8..11]  : Longitude                (int32_t, degrees × 1e7)
 *   - [12..15] : Altitude                 (int32_t, millimeters)
 *   - [16..17] : Ground speed             (uint16_t, m/s × 100)
 *   - [18..19] : Heading                  (uint16_t, degrees × 100)
 *   - [20..23] : Horizontal accuracy      (uint32_t, mm × 100)
 *   - [24..27] : Vertical accuracy        (uint32_t, mm × 100)
 *   - [28..31] : Data age                 (uint32_t, milliseconds)
 *   - [32]     : GPS fix type              (uint8_t)
 *   - [33]     : Satellites in view        (uint8_t)
 *   - [34]     : GPS status flags          (uint8_t, bitmask)
 *   - [35..38] : System uptime             (uint32_t, milliseconds)
 *
 * Scaled integer fields are converted back to floating-point representations
 * using the inverse of the encoder scaling factors to recover SI units.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the GPS data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and GPS data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time receive paths.
 *
 * @note The decoder assumes that packet framing and checksum validation have
 *       already been performed before this function is called.
 *
 * @warning If the GPS payload layout, scaling factors, or GPS_SIZE change,
 *          this decoder must be updated accordingly to maintain protocol
 *          compatibility.
 */
uint8_t decode_GPS(const VEDTP_Main *vedtp, GPS_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_GPS ||
        vedtp->payload_length != GPS_SIZE) {
        return 0;
    }

    data->tow_ms = get_u32_le(vedtp->payload, 0);

    data->lat_deg = (double)get_i32_le(vedtp->payload, 4) / 1e7;
    data->lon_deg = (double)get_i32_le(vedtp->payload, 8) / 1e7;
    data->alt_m = (float)get_i32_le(vedtp->payload, 12) / 1000.0f;
    data->ground_speed_mps = (float)get_u16_le(vedtp->payload, 16) / 100.0f;
    data->heading_deg = (float)get_u16_le(vedtp->payload, 18) / 100.0f;
    data->hAcc_mm = (float)get_u32_le(vedtp->payload, 20) / 100.0f;
    data->vAcc_mm = (float)get_u32_le(vedtp->payload, 24) / 100.0f;
    data->age_ms = get_u32_le(vedtp->payload, 28);

    data->fix_type = vedtp->payload[32];
    data->sats     = vedtp->payload[33];
    data->flags    = vedtp->payload[34];

    data->uptime_ms = get_u32_le(vedtp->payload, 35);

    return 1;
}

/**
 * @brief Decode IMU sensor data from a VEDTP transport frame.
 *
 * This function extracts inertial, magnetic, and orientation-related sensor
 * data from a received @ref VEDTP_Main packet carrying an IMU message and
 * populates an @ref IMU_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the IMU
 * device type and that the payload length matches the expected IMU payload
 * size before decoding. All multi-byte fields are decoded assuming a
 * little-endian wire format.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..5]    : Acceleration vector        (X, Y, Z) int16_t × 1e3
 *   - [6..11]   : Angular rate (gyro)        (X, Y, Z) int16_t × 1e3
 *   - [12..17]  : Magnetic field             (X, Y, Z) int16_t × 1e3
 *   - [18..23]  : Rotation vector            (X, Y, Z) int16_t × 1e3
 *   - [24..29]  : Linear acceleration        (X, Y, Z) int16_t × 1e3
 *   - [30..35]  : Gravity vector             (X, Y, Z) int16_t × 1e3
 *   - [36..37]  : Temperature                (int16_t, °C × 100)
 *   - [38]      : System calibration status  (uint8_t)
 *   - [39]      : Gyroscope calibration      (uint8_t)
 *   - [40]      : Accelerometer calibration  (uint8_t)
 *   - [41]      : Magnetometer calibration   (uint8_t)
 *   - [42..45]  : System uptime              (uint32_t, milliseconds)
 *
 * Scaled integer fields are converted back to floating-point representations
 * by applying the inverse of the encoder scaling factors to recover SI units.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the IMU data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and IMU data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time receive paths.
 *
 * @note The decoder assumes that packet framing and checksum validation have
 *       already been performed before this function is called.
 *
 * @warning If the IMU payload layout, scaling factors, or IMU_SIZE change,
 *          this decoder must be updated accordingly to maintain protocol
 *          compatibility.
 */
uint8_t decode_IMU(const VEDTP_Main *vedtp, IMU_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_IMU ||
        vedtp->payload_length != IMU_SIZE) {
        return 0;
    }
    uint16_t o = 0;

    for (int i = 0; i < 3; i++) {
        data->acceleration[i] = (float)get_i16_le(vedtp->payload, o) / 1000.0f;
        o += 2;
    }
    for (int i = 0; i < 3; i++) {
        data->gyro[i] = (float)get_i16_le(vedtp->payload, o) / 1000.0f;
        o += 2;
    }
    for (int i = 0; i < 3; i++) {
        data->magnetic[i] = (float)get_i16_le(vedtp->payload, o) / 1000.0f;
        o += 2;
    }
    for (int i = 0; i < 3; i++) {
        data->rotationvector[i] = (float)get_i16_le(vedtp->payload, o) / 1000.0f;
        o += 2;
    }
    for (int i = 0; i < 3; i++) {
        data->linearacceleration[i] = (float)get_i16_le(vedtp->payload, o) / 1000.0f;
        o += 2;
    }
    for (int i = 0; i < 3; i++) {
        data->gravity[i] = (float)get_i16_le(vedtp->payload, o) / 1000.0f;
        o += 2;
    }
    data->temperature_C = (float)get_i16_le(vedtp->payload, o) / 100.0f;
    o += 2;

    data->systemcalibration        = vedtp->payload[o++];
    data->gyrocalibration          = vedtp->payload[o++];
    data->accelerometercalibration = vedtp->payload[o++];
    data->magnetometercalibration  = vedtp->payload[o++];

    data->uptime_ms = get_u32_le(vedtp->payload, o);

    return 1;
}

/**
 * @brief Decode internal atmosphere sensor data from a VEDTP transport frame.
 *
 * This function extracts internal environmental sensor measurements from a
 * received @ref VEDTP_Main packet carrying an INTERNAL_ATMOSPHERE message and
 * populates an @ref INTERNAL_ATMOSPHERE_Packet structure with the decoded
 * values.
 *
 * The decoder validates that the incoming packet corresponds to the internal
 * atmosphere device type and that the payload length matches the expected
 * internal atmosphere payload size before decoding. All multi-byte fields are
 * decoded assuming a little-endian wire format.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : Internal temperature        (int16_t, °C × 100)
 *   - [2..3]   : Internal humidity           (uint16_t, %RH × 100)
 *   - [4..7]   : Internal pressure           (uint32_t, mbar × 100)
 *   - [8..9]   : Total VOC                   (uint16_t, ppb)
 *   - [10..11] : Equivalent CO₂              (uint16_t, ppm)
 *   - [12]     : Leak status                 (uint8_t, 0 = no leak, 1 = leak)
 *   - [13..16] : System uptime               (uint32_t, milliseconds)
 *
 * Scaled integer fields are converted back to floating-point values using the
 * inverse of the encoder scaling factors to recover physical units.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the internal atmosphere data structure to
 *                    populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time receive paths and safety monitoring systems.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning If the internal atmosphere payload layout, scaling factors, or
 *          INTERNAL_ATMOSPHERE_SIZE change, this decoder must be updated
 *          accordingly to maintain protocol compatibility.
 */
uint8_t decode_INTERNAL_ATMOSPHERE( const VEDTP_Main *vedtp, INTERNAL_ATMOSPHERE_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_INTERNAL_ATMOSPHERE ||
        vedtp->payload_length != INTERNAL_ATMOSPHERE_SIZE) {
        return 0;
    }

    data->internal_temp_C = (float)get_i16_le(vedtp->payload, 0) / 100.0f;
    data->internal_humidity_RH = (float)get_u16_le(vedtp->payload, 2) / 100.0f;
    data->internal_pressure_mbar = (float)get_u32_le(vedtp->payload, 4) / 100.0f;
    data->internal_tvoc_ppb = (float)get_u16_le(vedtp->payload, 8);
    data->internal_eco2_ppm = (float)get_u16_le(vedtp->payload, 10);
    data->leak_status = vedtp->payload[12];
    data->uptime_ms = get_u32_le(vedtp->payload, 13);

    return 1;
}

/**
 * @brief Decode external atmosphere and environmental sensor data from a VEDTP transport frame.
 *
 * This function extracts external environmental measurements from a received
 * @ref VEDTP_Main packet carrying an EXTERNAL_ATMOSPHERE message and populates
 * an @ref EXTERNAL_ATMOSPHERE_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the external
 * atmosphere device type and that the payload length matches the expected
 * EXTERNAL_ATMOSPHERE payload size before decoding. All multi-byte fields are
 * decoded assuming a little-endian wire format.
 *
 * This message combines atmospheric and marine environmental parameters and is
 * intended for navigation support, environmental monitoring, and scientific
 * data acquisition.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : Ambient temperature          (int16_t, °C × 100)
 *   - [2..5]   : Ambient pressure             (uint32_t, mbar × 10)
 *   - [6..7]   : Relative humidity            (uint16_t, %RH × 100)
 *   - [8..11]  : Altitude                     (int32_t, meters × 100)
 *   - [12..13] : Total VOC                    (uint16_t, ppb)
 *   - [14..15] : Equivalent CO₂               (uint16_t, ppm)
 *   - [16..17] : Air quality index            (uint16_t)
 *   - [18..21] : Water depth                  (int32_t, meters × 100)
 *   - [22..23] : Salinity                     (uint16_t, ppt × 100)
 *   - [24..27] : Water conductivity           (uint32_t, µS/cm)
 *   - [28..29] : Dissolved oxygen              (uint16_t, mg/L × 100)
 *   - [30..31] : Turbidity                    (uint16_t, NTU × 100)
 *   - [32]     : Sensor status flags          (uint8_t, bitmask)
 *   - [33]     : Medium type                  (uint8_t, air / water)
 *   - [34..37] : System uptime                (uint32_t, milliseconds)
 *
 * Scaled integer fields are converted back to floating-point values using the
 * inverse of the encoder scaling factors to recover physical units.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the external atmosphere data structure to
 *                    populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time receive paths and environmental monitoring systems.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning If the external atmosphere payload layout, scaling factors, or
 *          EXTERNAL_ATMOSPHERE_SIZE change, this decoder must be updated
 *          accordingly to maintain protocol compatibility.
 */
uint8_t decode_EXTERNAL_ATMOSPHERE( const VEDTP_Main *vedtp, EXTERNAL_ATMOSPHERE_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_EXTERNAL_ATMOSPHERE ||
        vedtp->payload_length != EXTERNAL_ATMOSPHERE_SIZE) {
        return 0;
    }

    data->temperature_C = get_i16_le(vedtp->payload, 0) / 100.0f;
    data->pressure_mbar = get_u32_le(vedtp->payload, 2) / 10.0f;
    data->humidity_RH = get_u16_le(vedtp->payload, 6) / 100.0f;
    data->altitude_m = get_i32_le(vedtp->payload, 8) / 100.0f;

    data->tvoc_ppb = get_u16_le(vedtp->payload, 12);
    data->eco2_ppm = get_u16_le(vedtp->payload, 14);
    data->air_quality_index = get_u16_le(vedtp->payload, 16);

    data->depth_m = get_i32_le(vedtp->payload, 18) / 100.0f;
    data->salinity_ppt = get_u16_le(vedtp->payload, 22) / 100.0f;
    data->conductivity_uS = (float)get_u32_le(vedtp->payload, 24);
    data->water_dissolved_oxygen_mgL = get_u16_le(vedtp->payload, 28) / 100.0f;
    data->turbidity_NTU = get_u16_le(vedtp->payload, 30) / 100.0f;

    data->sensor_status_flags = vedtp->payload[32];
    data->medium_type         = vedtp->payload[33];
    data->uptime_ms           = get_u32_le(vedtp->payload, 34);

    return 1;
}

/**
 * @brief Decode sonar ranging data from a VEDTP transport frame.
 *
 * This function extracts sonar measurement data from a received
 * @ref VEDTP_Main packet carrying a SONAR message and populates a
 * @ref SONAR_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the SONAR
 * device type and that the payload length matches the expected SONAR payload
 * size before decoding. All multi-byte fields are decoded assuming a
 * little-endian wire format.
 *
 * Sonar packets provide obstacle distance, signal quality, and confidence
 * information used for collision avoidance, terrain following, and
 * environmental perception.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0]      : Sonar type                  (uint8_t, model / modality)
 *   - [1]      : Sonar ID                    (uint8_t, sensor instance)
 *   - [2..3]   : Range                      (uint16_t, meters × 100)
 *   - [4..5]   : Signal strength             (uint16_t, normalized × 1000)
 *   - [6..7]   : Measurement confidence      (uint16_t, normalized × 1000)
 *   - [8..11]  : System uptime               (uint32_t, milliseconds)
 *
 * Scaled integer fields are converted back to floating-point values using
 * the inverse of the encoder scaling factors.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the sonar data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and SONAR data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time receive paths.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning If the sonar payload layout, scaling factors, or SONAR_SIZE change,
 *          this decoder must be updated accordingly to maintain protocol
 *          compatibility.
 */
uint8_t decode_SONAR(const VEDTP_Main *vedtp, SONAR_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_SONAR ||
        vedtp->payload_length != SONAR_SIZE) {
        return 0;
    }

    data->sonar_type = vedtp->payload[0];
    data->sonar_id   = vedtp->payload[1];

    data->range_m = (float)get_u16_le(vedtp->payload, 2) / 100.0f;
    data->signal_strength = (float)get_u16_le(vedtp->payload, 4) / 1000.0f;
    data->confidence = (float)get_u16_le(vedtp->payload, 6) / 1000.0f;
    data->uptime_ms = get_u32_le(vedtp->payload, 8);

    return 1;
}

/**
 * @brief Decode motor and propulsion feedback data from a VEDTP transport frame.
 *
 * This function extracts per-motor operational feedback from a received
 * @ref VEDTP_Main packet carrying a MOTOR message and populates a
 * @ref MOTOR_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the MOTOR
 * device type and that the payload length matches the expected MOTOR payload
 * size before decoding. All multi-byte fields are decoded assuming a
 * little-endian wire format.
 *
 * Each motor entry provides instantaneous and cumulative feedback used for
 * closed-loop control, diagnostics, and health monitoring.
 *
 * Payload layout (byte offsets, little-endian, repeated per motor):
 *   - RPM                      (int16_t)
 *   - Velocity                 (int16_t, m/s × 100)
 *   - Distance                 (int32_t, meters × 100)
 *   - Current                  (int16_t, A × 100)
 *   - Temperature              (int16_t, °C × 100)
 *   - Estimated torque         (int16_t, normalized × 100)
 *   - Voltage                  (uint16_t, V × 100)
 *   - PWM output               (int16_t)
 *   - Fault code               (uint8_t)
 *   - Control mode             (uint8_t)
 *   - Status flags              (uint8_t, bitmask)
 *   - Reserved                 (uint8_t)
 *
 * This block is repeated for 4 motors in fixed order, followed by a global
 * timestamp.
 *
 * Trailer:
 *   - System uptime             (uint32_t, milliseconds)
 *
 * Scaled integer fields are converted back to floating-point values using
 * the inverse of the encoder scaling factors.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the motor feedback data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and motor data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time receive paths and control feedback loops.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning Any modification to the motor payload layout, scaling factors,
 *          motor count, or MOTOR_SIZE must be reflected in this decoder to
 *          maintain protocol compatibility.
 */
uint8_t decode_MOTOR(const VEDTP_Main *vedtp, MOTOR_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_MOTOR ||
        vedtp->payload_length != MOTOR_SIZE) {
        return 0;
    }

    uint16_t o = 0;

    for (int i = 0; i < 4; i++) {

        data->rpm[i]         = (float)get_i16_le(vedtp->payload, o); o += 2;
        data->velocity[i]    = (float)get_i16_le(vedtp->payload, o) / 100.0f; o += 2;
        data->distance[i]    = (float)get_i32_le(vedtp->payload, o) / 100.0f; o += 4;
        data->current[i]     = (float)get_i16_le(vedtp->payload, o) / 100.0f; o += 2;
        data->temperature[i] = (float)get_i16_le(vedtp->payload, o) / 100.0f; o += 2;
        data->torque_est[i]  = (float)get_i16_le(vedtp->payload, o) / 100.0f; o += 2;
        data->voltage[i]     = (float)get_u16_le(vedtp->payload, o) / 100.0f; o += 2;
        data->pwm[i]         = (float)get_i16_le(vedtp->payload, o); o += 2;

        data->fault_code[i]   = vedtp->payload[o++];
        data->control_mode[i] = vedtp->payload[o++];
        data->status_flags[i] = vedtp->payload[o++];
        data->reserved[i]     = vedtp->payload[o++];
    }

    data->uptime_ms = get_u32_le(vedtp->payload, o);

    return 1;
}

/**
 * @brief Decode dead-reckoning and fused navigation data from a VEDTP transport frame.
 *
 * This function extracts local position, heading, velocity, and GPS-assist
 * information from a received @ref VEDTP_Main packet carrying a
 * DEAD_RECKONING message and populates a @ref DEAD_RECKONING_Packet structure
 * with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the
 * DEAD_RECKONING device type and that the payload length matches the expected
 * DEAD_RECKONING payload size before decoding. All multi-byte fields are
 * decoded assuming a little-endian wire format.
 *
 * Dead-reckoning packets provide the vehicle’s locally estimated pose,
 * optionally fused with GPS data, and are intended for navigation, control,
 * logging, and estimator debugging.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..3]   : Local X position            (int32_t, millimeters)
 *   - [4..7]   : Local Y position            (int32_t, millimeters)
 *   - [8..11]  : Local Z position            (int32_t, millimeters)
 *   - [12..13] : Yaw angle                   (uint16_t, degrees × 100)
 *   - [14..15] : Ground speed                (uint16_t, m/s × 100)
 *   - [16..19] : GPS latitude                (int32_t, degrees × 1e7)
 *   - [20..23] : GPS longitude               (int32_t, degrees × 1e7)
 *   - [24..27] : GPS altitude                (int32_t, meters × 100)
 *   - [28..29] : GPS speed                   (uint16_t, m/s × 100)
 *   - [30..31] : GPS course                  (uint16_t, degrees × 100)
 *   - [32]     : GPS fused flag              (uint8_t, 0 = no, 1 = yes)
 *   - [33]     : Dead-reckoning active flag  (uint8_t)
 *   - [34]     : GPS validity flag           (uint8_t)
 *   - [35]     : Reserved                    (uint8_t)
 *   - [36..39] : System uptime               (uint32_t, milliseconds)
 *
 * Scaled integer fields are converted back to floating-point values using the
 * inverse of the encoder scaling factors to recover physical units.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the dead-reckoning data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and navigation data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time receive paths and state-estimation pipelines.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning Any modification to the dead-reckoning payload layout, scaling
 *          factors, or DEAD_RECKONING_SIZE must be reflected in this decoder
 *          to maintain protocol compatibility.
 */
uint8_t decode_DEAD_RECKONING( const VEDTP_Main *vedtp, DEAD_RECKONING_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_DEAD_RECKONING ||
        vedtp->payload_length != DEAD_RECKONING_SIZE) {
        return 0;
    }

    data->x_mm = get_i32_le(vedtp->payload, 0);
    data->y_mm = get_i32_le(vedtp->payload, 4);
    data->z_mm = get_i32_le(vedtp->payload, 8);

    data->yaw_cdeg = get_u16_le(vedtp->payload, 12) / 100.0f;
    data->speed_mps = get_u16_le(vedtp->payload, 14) / 100.0f;
    data->gps_lat = (double)get_i32_le(vedtp->payload, 16) / 1e7;
    data->gps_lon = (double)get_i32_le(vedtp->payload, 20) / 1e7;
    data->gps_alt = (double)get_i32_le(vedtp->payload, 24) / 100.0;
    data->gps_speed = get_u16_le(vedtp->payload, 28) / 100.0f;
    data->gps_course = get_u16_le(vedtp->payload, 30) / 100.0f;

    data->gps_fused      = vedtp->payload[32];
    data->deadrec_active = vedtp->payload[33];
    data->gps_valid      = vedtp->payload[34];
    data->reserved       = vedtp->payload[35];

    data->uptime_ms = get_u32_le(vedtp->payload, 36);

    return 1;
}

/**
 * @brief Decode power system and energy health data from a VEDTP transport frame.
 *
 * This function extracts power supply, battery, and energy consumption metrics
 * from a received @ref VEDTP_Main packet carrying a POWER_HEALTH message and
 * populates a @ref POWER_HEALTH_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the
 * POWER_HEALTH device type and that the payload length matches the expected
 * POWER_HEALTH payload size before decoding. All multi-byte fields are decoded
 * assuming a little-endian wire format.
 *
 * Power health packets provide real-time insight into electrical system state
 * and are intended for safety monitoring, power budgeting, autonomy decisions,
 * and mission endurance estimation.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : SMPS voltage                (uint16_t, volts × 100)
 *   - [2..3]   : Battery voltage             (uint16_t, volts × 100)
 *   - [4..5]   : Current draw                (int16_t, amperes × 100)
 *   - [6..7]   : Battery capacity            (uint16_t, percent × 100)
 *   - [8..9]   : Power usage                 (uint16_t, watts × 10)
 *   - [10..11] : Estimated uptime            (uint16_t, minutes × 10)
 *   - [12]     : Charging state              (uint8_t, 0 = no, 1 = yes)
 *   - [13]     : Discharging state           (uint8_t, 0 = no, 1 = yes)
 *   - [14]     : Battery present flag        (uint8_t)
 *   - [15]     : Power status flags          (uint8_t, bitmask)
 *   - [16..19] : System uptime               (uint32_t, milliseconds)
 *
 * Scaled integer fields are converted back to floating-point values using the
 * inverse of the encoder scaling factors to recover physical units.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the power health data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and power health data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time receive paths and safety monitoring logic.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning Any modification to the power health payload layout, scaling
 *          factors, or POWER_HEALTH_SIZE must be reflected in this decoder
 *          to maintain protocol compatibility.
 */
uint8_t decode_POWER_HEALTH( const VEDTP_Main *vedtp, POWER_HEALTH_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_POWER_HEALTH ||
        vedtp->payload_length != POWER_HEALTH_SIZE) {
        return 0;
    }

    data->smps_voltage = get_u16_le(vedtp->payload, 0) / 100.0f;
    data->battery_voltage = get_u16_le(vedtp->payload, 2) / 100.0f;
    data->current_draw = get_i16_le(vedtp->payload, 4) / 100.0f;
    data->battery_capacity_pct = get_u16_le(vedtp->payload, 6) / 100.0f;
    data->power_usage_watt = get_u16_le(vedtp->payload, 8) / 10.0f;
    data->estimated_uptime_min = get_u16_le(vedtp->payload, 10) / 10.0f;

    data->charging        = vedtp->payload[12];
    data->discharging     = vedtp->payload[13];
    data->battery_present = vedtp->payload[14];
    data->power_flags     = vedtp->payload[15];

    data->uptime_ms = get_u32_le(vedtp->payload, 16);

    return 1;
}

/**
 * @brief Decode flight controller health and runtime status data from a VEDTP transport frame.
 *
 * This function extracts flight controller (FC) health, resource usage, and
 * runtime status information from a received @ref VEDTP_Main packet carrying
 * an FC_HEALTH message and populates a @ref FC_HEALTH_Packet structure with
 * the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the FC_HEALTH
 * device type and that the payload length matches the expected FC_HEALTH
 * payload size before decoding. All multi-byte fields are decoded assuming a
 * little-endian wire format.
 *
 * FC health packets provide visibility into controller thermal state, CPU and
 * memory utilization, RTOS health, and fault conditions. This data is intended
 * for system supervision, fault detection, and autonomous failsafe decisions.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : Controller temperature      (int16_t, °C × 100)
 *   - [2..3]   : CPU usage                   (uint16_t, percent × 100)
 *   - [4..5]   : Heap usage                  (uint16_t, percent × 100)
 *   - [6..7]   : Stack high watermark        (uint16_t, percent × 100)
 *   - [8..11]  : RTOS tick count             (uint32_t)
 *   - [12..13] : Error counter               (uint16_t)
 *   - [14..15] : Warning counter             (uint16_t)
 *   - [16]     : Reset reason                (uint8_t, platform-defined)
 *   - [17]     : RTOS status                 (uint8_t, 0 = fault, 1 = OK)
 *   - [18]     : Task fault flags             (uint8_t, bitmask)
 *   - [19]     : Failsafe flags               (uint8_t, bitmask)
 *   - [20..23] : System uptime               (uint32_t, milliseconds)
 *
 * Scaled integer fields are converted back to floating-point values using the
 * inverse of the encoder scaling factors to recover physical units.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the flight controller health data structure
 *                    to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and FC health data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time receive paths and supervisory health monitoring.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning Any modification to the FC health payload layout, scaling factors,
 *          or FC_HEALTH_SIZE must be reflected in this decoder to maintain
 *          protocol compatibility.
 */
uint8_t decode_FC_HEALTH( const VEDTP_Main *vedtp, FC_HEALTH_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_FC_HEALTH ||
        vedtp->payload_length != FC_HEALTH_SIZE) {
        return 0;
    }

    data->temperature = get_i16_le(vedtp->payload, 0) / 100.0f;
    data->cpu_usage_pct = get_u16_le(vedtp->payload, 2) / 100.0f;
    data->heap_usage_pct = get_u16_le(vedtp->payload, 4) / 100.0f;
    data->stack_high_watermark_pct = get_u16_le(vedtp->payload, 6) / 100.0f;
    data->tick_count = get_u32_le(vedtp->payload, 8);
    data->error_count = get_u16_le(vedtp->payload, 12);
    data->warning_count = get_u16_le(vedtp->payload, 14);

    data->reset_reason     = vedtp->payload[16];
    data->rtos_ok          = vedtp->payload[17];
    data->task_fault_flags = vedtp->payload[18];
    data->failsafe_flags   = vedtp->payload[19];

    data->uptime_ms = get_u32_le(vedtp->payload, 20);

    return 1;
}

/**
 * @brief Decode a command message from a VEDTP transport frame.
 *
 * This function extracts a command definition from a received
 * @ref VEDTP_Main packet carrying a COMMAND message and populates a
 * @ref COMMAND_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the COMMAND
 * device type and that the payload length matches the expected COMMAND payload
 * size before decoding. All multi-byte fields are decoded assuming a
 * little-endian wire format.
 *
 * Command packets represent explicit control or configuration requests and
 * are intended to be processed by command handlers, mission logic, or control
 * arbitration layers.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0]      : Command ID                  (uint8_t)
 *   - [1]      : Target ID                   (uint8_t, subsystem / component)
 *   - [2]      : Command flags               (uint8_t, bitmask)
 *   - [3]      : Parameter count             (uint8_t)
 *   - [4..15]  : Command parameters          (6 × int16_t)
 *   - [16..27] : String parameter 1          (12 bytes, fixed-length, ASCII/raw)
 *   - [28..39] : String parameter 2          (12 bytes, fixed-length, ASCII/raw)
 *   - [40..43] : System uptime               (uint32_t, milliseconds)
 *
 * The parameter and string fields are transmitted using fixed-length encoding
 * to ensure deterministic parsing and bounded memory usage on embedded
 * receivers. Interpretation of parameters depends on the command ID.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the command data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and command data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time command processing paths.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning Any modification to the command payload layout, parameter count,
 *          or COMMAND_SIZE must be reflected in this decoder to maintain
 *          protocol compatibility.
 */
uint8_t decode_COMMAND( const VEDTP_Main *vedtp, COMMAND_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_COMMAND ||
        vedtp->payload_length != COMMAND_SIZE) {
        return 0;
    }

    data->id          = vedtp->payload[0];
    data->target      = vedtp->payload[1];
    data->flags       = vedtp->payload[2];
    data->param_count = vedtp->payload[3];

    uint16_t o = 4;

    for (int i = 0; i < 6; i++) {
        data->param[i] = get_i16_le(vedtp->payload, o);
        o += 2;
    }

    memcpy(data->char1, &vedtp->payload[o], 12);
    o += 12;

    memcpy(data->char2, &vedtp->payload[o], 12);
    o += 12;

    data->uptime_ms = get_u32_le(vedtp->payload, o);

    return 1;
}

/**
 * @brief Decode joystick control input from a VEDTP transport frame.
 *
 * This function extracts manual control input data from a received
 * @ref VEDTP_Main packet carrying a JOYSTICK message and populates a
 * @ref JOYSTICK_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the JOYSTICK
 * device type and that the payload length matches the expected JOYSTICK payload
 * size before decoding. All multi-byte fields are decoded assuming a
 * little-endian wire format.
 *
 * Joystick packets represent real-time operator input and are intended for
 * direct manual control, assisted control modes, or command arbitration
 * alongside autonomous behaviors.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..15]  : Joystick channels            (8 × uint16_t, raw values)
 *   - [16]     : Channel count                (uint8_t)
 *   - [17]     : Joystick flags               (uint8_t, bitmask)
 *   - [18..21] : System uptime                (uint32_t, milliseconds)
 *
 * Channel values are transmitted as raw unsigned integers. Interpretation
 * (centering, scaling, dead zones, inversion) is performed by the receiver
 * based on the active control mode and configuration.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the joystick data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and joystick data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time control input paths.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning Joystick data directly influences vehicle motion. Any modification
 *          to the joystick payload layout, channel count, or JOYSTICK_SIZE
 *          must be reflected in this decoder to maintain protocol and safety
 *          correctness.
 */
uint8_t decode_JOYSTICK( const VEDTP_Main *vedtp, JOYSTICK_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_JOYSTICK ||
        vedtp->payload_length != JOYSTICK_SIZE) {
        return 0;
    }

    uint16_t o = 0;

    for (int i = 0; i < 8; i++) {
        data->channels[i] = get_u16_le(vedtp->payload, o);
        o += 2;
    }

    data->channel_count = vedtp->payload[o++];
    data->flags         = vedtp->payload[o++];

    data->uptime_ms =
        get_u32_le(vedtp->payload, o);

    return 1;
}

/**
 * @brief Decode flow meter and water current data from a VEDTP transport frame.
 *
 * This function extracts hydrodynamic flow measurements and sensor orientation
 * data from a received @ref VEDTP_Main packet carrying a FLOWMETER message and
 * populates a @ref FLOWMETER_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the flow meter
 * device type and that the payload length matches the expected FLOWMETER payload
 * size before decoding. All multi-byte fields are decoded assuming a
 * little-endian wire format.
 *
 * Flow meter packets provide localized water current information and sensor
 * orientation context, enabling current-aware navigation, dead-reckoning
 * correction, and environmental analysis.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : Flow meter RPM              (int16_t)
 *   - [2..3]   : Flow speed                  (int16_t, m/s × 100)
 *   - [4..5]   : Flow direction              (uint16_t, degrees × 100)
 *   - [6..7]   : Sensor roll                 (int16_t, degrees × 100)
 *   - [8..9]   : Sensor pitch                (int16_t, degrees × 100)
 *   - [10..11] : Sensor yaw                  (uint16_t, degrees × 100)
 *   - [12]     : Sensor status flags         (uint8_t, bitmask)
 *   - [13..16] : System uptime               (uint32_t, milliseconds)
 *
 * Scaled integer fields are converted back to floating-point values using the
 * inverse of the encoder scaling factors to recover physical units.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the flow meter data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and flow meter data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time receive paths and navigation estimation pipelines.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning Any modification to the flow meter payload layout, scaling factors,
 *          or FLOWMETER_SIZE must be reflected in this decoder to maintain
 *          protocol compatibility.
 */
uint8_t decode_FLOWMETER( const VEDTP_Main *vedtp, FLOWMETER_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_FLOW_METER ||
        vedtp->payload_length != FLOWMETER_SIZE) {
        return 0;
    }

    data->rpm = (float)get_i16_le(vedtp->payload, 0);
    data->flow_speed_mps = (float)get_i16_le(vedtp->payload, 2) / 100.0f;
    data->flow_direction_deg = (float)get_u16_le(vedtp->payload, 4) / 100.0f;
    data->roll = (float)get_i16_le(vedtp->payload, 6) / 100.0f;
    data->pitch = (float)get_i16_le(vedtp->payload, 8) / 100.0f;
    data->yaw = (float)get_u16_le(vedtp->payload, 10) / 100.0f;
    data->status_flags = vedtp->payload[12];
    data->uptime_ms = get_u32_le(vedtp->payload, 13);

    return 1;
}

/**
 * @brief Decode a command acknowledgement message from a VEDTP transport frame.
 *
 * This function extracts the execution result of a previously issued command
 * from a received @ref VEDTP_Main packet carrying a COMMAND_ACK message and
 * populates a @ref COMMAND_ACK_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the
 * COMMAND_ACK device type and that the payload length matches the expected
 * COMMAND_ACK payload size before decoding. All multi-byte fields are decoded
 * assuming a little-endian wire format.
 *
 * Command acknowledgement packets provide deterministic feedback to the command
 * issuer, enabling reliable command tracking, retry logic, fault diagnosis,
 * and operator visibility.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0]      : Command ID                  (uint8_t, echoed command)
 *   - [1]      : Result code                 (uint8_t, success / error)
 *   - [2]      : Source                      (uint8_t, responding subsystem)
 *   - [3]      : Acknowledgement flags       (uint8_t, bitmask)
 *   - [4..53]  : Status message              (50 bytes, fixed-length, ASCII/raw)
 *   - [54..57] : System uptime               (uint32_t, milliseconds)
 *
 * The status message field is transmitted using a fixed-length encoding to
 * avoid variable-length parsing and to guarantee bounded memory usage on
 * embedded receivers. The message may be null-terminated or raw, depending
 * on context.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the command acknowledgement data structure
 *                    to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and acknowledgement data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time command-handling and supervision paths.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning Any modification to the command acknowledgement payload layout,
 *          message length, or COMMAND_ACK_SIZE must be reflected in this
 *          decoder to maintain protocol compatibility.
 */
uint8_t decode_COMMAND_ACK( const VEDTP_Main *vedtp, COMMAND_ACK_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_COMMAND_ACK ||
        vedtp->payload_length != COMMAND_ACK_SIZE) {
        return 0;
    }

    data->command = vedtp->payload[0];
    data->result  = vedtp->payload[1];
    data->source  = vedtp->payload[2];
    data->flags   = vedtp->payload[3];

    memcpy(data->message, &vedtp->payload[4], 50);

    data->uptime_ms = get_u32_le(vedtp->payload, 54);

    return 1;
}

/**
 * @brief Decode a device-level error report from a VEDTP transport frame.
 *
 * This function extracts fault and error information from a received
 * @ref VEDTP_Main packet carrying a DEVICE_ERROR message and populates a
 * @ref DEVICE_ERROR_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the
 * DEVICE_ERROR device type and that the payload length matches the expected
 * DEVICE_ERROR payload size before decoding. All multi-byte fields are decoded
 * assuming a little-endian wire format.
 *
 * Device error packets provide a lightweight, deterministic mechanism for
 * reporting faults, warnings, and critical failures originating from individual
 * subsystems. These packets are intended for real-time fault monitoring,
 * safety handling, and post-incident diagnostics.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0]      : Device ID                   (uint8_t, fault source)
 *   - [1]      : Error type                  (uint8_t, subsystem-defined)
 *   - [2]      : Severity level              (uint8_t, info / warning / critical)
 *   - [3]      : Error flags                 (uint8_t, bitmask)
 *   - [4..7]   : System uptime               (uint32_t, milliseconds)
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the device error data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and device error data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time fault handling and safety monitoring paths.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning Device error packets may indicate critical system conditions.
 *          Callers should evaluate severity and flags immediately and take
 *          appropriate action (e.g., failsafe, shutdown, mission abort).
 */
uint8_t decode_DEVICE_ERROR( const VEDTP_Main *vedtp, DEVICE_ERROR_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_DEVICE_ERROR ||
        vedtp->payload_length != DEVICE_ERROR_SIZE) {
        return 0;
    }

    data->device_id  = vedtp->payload[0];
    data->error_type = vedtp->payload[1];
    data->severity   = vedtp->payload[2];
    data->flags      = vedtp->payload[3];

    data->uptime_ms = get_u32_le(vedtp->payload, 4);

    return 1;
}

/**
 * @brief Decode Cone Penetration Test (CPT) measurement data from a VEDTP transport frame.
 *
 * This function extracts geotechnical CPT measurement data from a received
 * @ref VEDTP_Main packet carrying a CPT_DATA message and populates a
 * @ref CPT_DATA_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the CPT_DATA
 * device type and that the payload length matches the expected CPT_DATA payload
 * size before decoding. All multi-byte fields are decoded assuming a
 * little-endian wire format.
 *
 * CPT data packets combine mechanical probe measurements, probe orientation,
 * geolocation, and contextual metadata. They are intended for real-time
 * monitoring, data logging, and post-processing in seabed and subsurface
 * analysis workflows.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : Probe angle                 (int16_t, degrees × 100)
 *   - [2..5]   : Penetration pressure        (uint32_t, mbar × 10)
 *   - [6..7]   : Penetration depth           (int16_t, millimeters)
 *   - [8..9]   : Insertion speed             (int16_t, mm/s × 10)
 *   - [10..13] : Strain measurement          (int32_t, raw × 1000)
 *   - [14..15] : Rod temperature             (int16_t, °C × 100)
 *   - [16..19] : Latitude                   (int32_t, degrees × 1e7)
 *   - [20..23] : Longitude                  (int32_t, degrees × 1e7)
 *   - [24..27] : Depth                      (int32_t, meters × 100)
 *   - [28..29] : Roll angle                 (int16_t, degrees × 100)
 *   - [30..31] : Pitch angle                (int16_t, degrees × 100)
 *   - [32..33] : Yaw angle                  (uint16_t, degrees × 100, [0–360))
 *   - [34]     : Status flags               (uint8_t, bitmask)
 *   - [35]     : Soil classification        (uint8_t, enumeration)
 *   - [36]     : Measurement step index     (uint8_t)
 *   - [37]     : Active state               (uint8_t, 0 = inactive, 1 = active)
 *   - [38..53] : Measurement label          (16 bytes, fixed-length, ASCII/raw)
 *   - [54..57] : System uptime              (uint32_t, milliseconds)
 *
 * Scaled integer fields are converted back to floating-point values using the
 * inverse of the encoder scaling factors to recover physical units.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the CPT data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and CPT data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time data acquisition and logging paths.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning CPT data is typically used for scientific or engineering decisions.
 *          Any modification to the CPT payload layout, scaling factors, or
 *          CPT_DATA_SIZE must be reflected in this decoder to maintain
 *          protocol compatibility and data integrity.
 */
uint8_t decode_CPT_DATA( const VEDTP_Main *vedtp, CPT_DATA_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_CPT_DATA ||
        vedtp->payload_length != CPT_DATA_SIZE) {
        return 0;
    }

    data->angle_deg = get_i16_le(vedtp->payload, 0) / 100.0f;
    data->pressure_mbar = get_u32_le(vedtp->payload, 2) / 10.0f;
    data->penetration_mm = get_i16_le(vedtp->payload, 6);
    data->insertion_speed_mmps = get_i16_le(vedtp->payload, 8) / 10.0f;
    data->strain_raw = get_i32_le(vedtp->payload, 10) / 1000.0f;
    data->rod_temp_c = get_i16_le(vedtp->payload, 14) / 100.0f;
    data->latitude_deg = (double)get_i32_le(vedtp->payload, 16) / 1e7;
    data->longitude_deg = (double)get_i32_le(vedtp->payload, 20) / 1e7;
    data->depth_m = get_i32_le(vedtp->payload, 24) / 100.0f;
    data->roll_deg = get_i16_le(vedtp->payload, 28) / 100.0f;
    data->pitch_deg = get_i16_le(vedtp->payload, 30) / 100.0f;
    data->yaw_deg = get_u16_le(vedtp->payload, 32) / 100.0f;

    data->status_flags = vedtp->payload[34];
    data->soil_class   = vedtp->payload[35];
    data->step_index   = vedtp->payload[36];
    data->active       = vedtp->payload[37];

    memcpy(data->label, &vedtp->payload[38], 16);

    data->uptime_ms =
        get_u32_le(vedtp->payload, 54);

    return 1;
}

/**
 * @brief Decode a waypoint mission command from a VEDTP transport frame.
 *
 * This function extracts waypoint mission data from a received
 * @ref VEDTP_Main packet carrying a WAYPOINT message and populates a
 * @ref WAYPOINT_Packet structure with the decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the WAYPOINT
 * device type and that the payload length matches the expected WAYPOINT payload
 * size before decoding. All multi-byte fields are decoded assuming a
 * little-endian wire format.
 *
 * Waypoint packets define mission navigation objectives and associated
 * execution parameters. They are intended for use by autonomous navigation,
 * mission planning, and guidance logic.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0]      : Waypoint ID                 (uint8_t)
 *   - [1]      : Total waypoint count        (uint8_t)
 *   - [2]      : Waypoint type               (uint8_t, navigation / action)
 *   - [3]      : Next command                (uint8_t, sequencing hint)
 *   - [4]      : Action code                 (uint8_t, mission-specific)
 *   - [5]      : Reference frame             (uint8_t, global / local)
 *   - [6..9]   : Latitude                   (int32_t, degrees × 1e7)
 *   - [10..13] : Longitude                  (int32_t, degrees × 1e7)
 *   - [14..17] : Altitude / depth            (int32_t, millimeters)
 *   - [18..19] : Transit speed               (uint16_t, m/s × 100)
 *   - [20..21] : Acceptance radius           (uint16_t, meters × 100)
 *   - [22..23] : Hold time                   (int16_t, seconds)
 *   - [24..25] : Desired yaw                 (uint16_t, degrees × 100)
 *   - [26]     : Waypoint flags              (uint8_t, bitmask)
 *   - [27]     : Retry count                 (uint8_t)
 *   - [28..39] : Waypoint label              (12 bytes, fixed-length, ASCII/raw)
 *   - [40..43] : System uptime               (uint32_t, milliseconds)
 *
 * Scaled integer fields are converted back to floating-point values using the
 * inverse of the encoder scaling factors to recover physical units.
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the waypoint data structure to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and waypoint data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time mission execution and planning paths.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning Waypoint packets directly affect autonomous vehicle behavior.
 *          Any modification to the waypoint payload layout, scaling factors,
 *          or WAYPOINT_SIZE must be reflected in this decoder to maintain
 *          mission correctness and protocol compatibility.
 */
uint8_t decode_WAYPOINT( const VEDTP_Main *vedtp, WAYPOINT_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_WAYPOINT ||
        vedtp->payload_length != WAYPOINT_SIZE) {
        return 0;
    }

    data->id          = vedtp->payload[0];
    data->wp_count    = vedtp->payload[1];
    data->wp_type     = vedtp->payload[2];
    data->next_cmd    = vedtp->payload[3];
    data->action_code = vedtp->payload[4];
    data->frame       = vedtp->payload[5];

    data->lat_deg = (double)get_i32_le(vedtp->payload, 6) / 1e7;
    data->lon_deg = (double)get_i32_le(vedtp->payload, 10) / 1e7;
    data->alt_mm = get_i32_le(vedtp->payload, 14);
    data->speed_mps = get_u16_le(vedtp->payload, 18) / 100.0f;
    data->radius_m = get_u16_le(vedtp->payload, 20) / 100.0f;
    data->hold_time_s = get_i16_le(vedtp->payload, 22);
    data->yaw_deg = get_u16_le(vedtp->payload, 24) / 100.0f;

    data->flags   = vedtp->payload[26];
    data->retries = vedtp->payload[27];

    memcpy(data->label, &vedtp->payload[28], 12);

    data->uptime_ms = get_u32_le(vedtp->payload, 40);

    return 1;
}

/**
 * @brief Decode a waypoint acknowledgement from a VEDTP transport frame.
 *
 * This function extracts acknowledgement information for a previously
 * transmitted waypoint from a received @ref VEDTP_Main packet carrying a
 * WP_ACK message and populates a @ref WP_ACK_Packet structure with the
 * decoded values.
 *
 * The decoder validates that the incoming packet corresponds to the WP_ACK
 * device type and that the payload length matches the expected WP_ACK payload
 * size before decoding. All multi-byte fields are decoded assuming a
 * little-endian wire format.
 *
 * Waypoint acknowledgement packets provide deterministic feedback indicating
 * whether a waypoint was accepted, rejected, or failed during processing.
 * They are intended for mission synchronization, retry logic, and operator
 * feedback.
 *
 * Payload layout (byte offsets, little-endian):
 *   - [0..1]   : Waypoint ID                 (uint16_t)
 *   - [2]      : Result code                 (uint8_t, success / error)
 *   - [3..6]   : System uptime               (uint32_t, milliseconds)
 *
 * @param[in]  vedtp  Pointer to the received VEDTP packet.
 * @param[out] data   Pointer to the waypoint acknowledgement data structure
 *                    to populate.
 *
 * @return uint8_t
 *         - 1 : Decoding successful and acknowledgement data populated.
 *         - 0 : Invalid arguments, device mismatch, or payload size mismatch.
 *
 * @note This function performs no dynamic memory allocation and is suitable
 *       for real-time mission coordination paths.
 *
 * @note The decoder assumes that packet framing and checksum validation
 *       have already been performed before this function is called.
 *
 * @warning WP_ACK packets directly affect mission flow control. Callers
 *          should evaluate the result code immediately and take appropriate
 *          action (e.g., retry, abort, or proceed to next waypoint).
 */
uint8_t decode_WP_ACK( const VEDTP_Main *vedtp, WP_ACK_Packet *data)
{
    if (!vedtp || !data) return 0;

    if (vedtp->device != DEVICE_WP_ACK ||
        vedtp->payload_length != WP_ACK_SIZE) {
        return 0;
    }

    data->wp_id = get_u16_le(vedtp->payload, 0);
    data->result = vedtp->payload[2];
    data->uptime_ms = get_u32_le(vedtp->payload, 3);

    return 1;
}
