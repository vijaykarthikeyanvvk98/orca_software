#ifndef LINK_H
#define LINK_H
#include <QObject>
#include<QTimer>
#include<QString>
#include<QByteArray>
#include<QDateTime>
#include "receiver.h"
#include "joystickcontroller.h"
#include <QTimer>
#include <QElapsedTimer>
#include "VEDTP.h"
class Link:public QObject
{
    Q_OBJECT
    Q_PROPERTY(float yaw_value READ yaw_value WRITE setYaw_value NOTIFY yawValueChanged)
    Q_PROPERTY(float pitch_value READ pitch_value WRITE setPitch_value NOTIFY pitchValueChanged)
    Q_PROPERTY(float roll_value READ roll_value WRITE setRoll_value NOTIFY rollValueChanged)
    Q_PROPERTY(quint64 uptime_ms_ahrs READ uptime_ms_ahrs_value WRITE setuptime_ms_ahrs_value NOTIFY uptimeChanged)
    Q_PROPERTY(float temperature READ temperature WRITE settemperature NOTIFY temperatureChanged)
    Q_PROPERTY(float depth READ depth WRITE setdepth NOTIFY depthChanged)
    Q_PROPERTY(float salinity READ salinity WRITE setsalinity NOTIFY salinityChanged)
    Q_PROPERTY(uint8_t device_id READ deviceId WRITE setDeviceId NOTIFY deviceIdChanged)
    Q_PROPERTY(uint8_t error_type READ errorType WRITE setErrorType NOTIFY errorTypeChanged)
    Q_PROPERTY(uint8_t severity READ error_severity WRITE setseverity NOTIFY severityChanged)
    Q_PROPERTY(uint8_t flags READ errorflags WRITE setFlags NOTIFY flagsChanged)
    Q_PROPERTY(quint64 uptime_ms_error READ uptime_mserror WRITE setuptimemserror NOTIFY uptimemserrorChanged)
    Q_PROPERTY(float motor_velocity_4 READ motorVelocity4 WRITE setMotorVelocity4 NOTIFY motorVelocity4Changed)
    Q_PROPERTY(float battery_voltage READ batteryVoltage WRITE setBatteryVoltage NOTIFY batteryVoltageChanged)
    Q_PROPERTY(float sonar_range READ sonarRange WRITE setSonarRange NOTIFY sonarRangeChanged)
    Q_PROPERTY(float sonar_confidence READ sonarConfidence WRITE setSonarConfidence NOTIFY sonarConfidenceChanged)
    Q_PROPERTY (uint8_t command READ commandValue WRITE setCommand NOTIFY commandChanged)
    Q_PROPERTY(uint8_t result READ resultValue WRITE setResult NOTIFY resultChanged)
    Q_PROPERTY(uint8_t source READ sourceValue WRITE setSource NOTIFY sourceChanged)
    Q_PROPERTY(uint8_t flags_ack READ flagsAck WRITE setFlagsAck NOTIFY flagsAckChanged)
    Q_PROPERTY(QString message READ messageValue WRITE setMessage NOTIFY messageChanged)
    Q_PROPERTY(quint64 uptime_ms_ack READ uptimeMsAck WRITE setUptimeMsAck NOTIFY uptimeMsAckChanged)
public:
    Link();
    ~Link();
    float yaw_value () const;
    float pitch_value () const;
    float roll_value () const;
    quint32 uptime_ms_ahrs_value () const;
    float temperature() const;
    float depth() const;
    float salinity() const;
    uint8_t deviceId() const;
    uint8_t errorType() const;
    uint8_t error_severity() const;
    uint8_t errorflags() const;
    quint64 uptime_mserror() const;
    float motorVelocity4() const;
    float batteryVoltage() const;
    float sonarRange() const;
    float sonarConfidence() const;
    uint8_t commandValue() const;
    uint8_t resultValue() const;
    uint8_t sourceValue() const;
    uint8_t flagsAck() const;
    QString messageValue() const;
    quint64 uptimeMsAck() const;
signals:
    void _invokeWriteBytes(QByteArray data);
    void data_processed(int device);
    void yawValueChanged(float value);
    void pitchValueChanged(float value);
    void rollValueChanged(float value);
    void uptimeChanged(uint);
    void sendBytes(QByteArray data);
    //void armedChanged()
    void ahrsChanged();
    void temperatureChanged(float value);
    void depthChanged(float value);
    void salinityChanged(float value);
    void deviceIdChanged(uint8_t);
    void errorTypeChanged(uint8_t);
    void severityChanged(uint8_t);
    void flagsChanged(uint8_t);
    void uptimemserrorChanged(quint64 );
    void motorVelocity4Changed(float);
    void batteryVoltageChanged(float);
    void sonarRangeChanged(float);
    void sonarConfidenceChanged(float);
    void commandChanged(uint8_t);
    void resultChanged(uint8_t);
    void sourceChanged(uint8_t);
    void flagsAckChanged(uint8_t);
    void messageChanged(const QString &);
    void uptimeMsAckChanged(quint64);
    //void sendBytes(const QByteArray&);
public slots:
    int mapvalue(double,int,int,int,int);
    void send_heart_beat();
    void create_directory();
    void mavlink_send_buffer(VEDTP_Main vedtp_send);
    void writeByteThreadSafe(const char *bytes, int length);
    void send_data(QByteArray buffer);
    void sendJoystickData();
    void send_data_to_process(QByteArray data);
    void data_to_be_updated(int);
    void heart_beat_parsing();
    void AHRS_parsing();
    void env_parsing();
    void error_parsing();
    void power_health_parsing();
    void motor_parsing();
    void sonar_parsing();
    void command_parsing();
    void setYaw_value(float value);
    void setPitch_value(float value);
    void setRoll_value(float value);
    void setuptime_ms_ahrs_value(quint64 value);
    void settemperature(float value);
    void setdepth(float value);
    void setsalinity(float value);
    void setDeviceId(uint8_t value);
    void setErrorType(uint8_t value);
    void setseverity(uint8_t value);
    void setFlags(uint8_t value);
    void setuptimemserror(quint64 value);
    void generateRandomData();
    void setMotorVelocity4(float value);
    void setBatteryVoltage(float value);
    void setSonarRange(float value);
    void setSonarConfidence(float value);
    void setCommand(uint8_t value);
    void setResult(uint8_t value);
    void setSource(uint8_t value);
    void setFlagsAck(uint8_t value);
    void setMessage(const QString &value);
    void setUptimeMsAck(quint64 value);
    void set_joystick_values(double, double);
    void set_joystick_values2(double, double);
    void set_joystick_values3(double, double);
    void set_joystick_values4(double, double);
    void joystick_activated(int mode, bool value);
    //void sendModeCommand(uint8_t mode);
    void sendMode(System_Mode mode);
    void setManualMode();
    void setHoldMode();
    void setAutoMode();
    void setRTSMode();
    void sendModeCommand(unsigned char mode);
private:
    Receiver receiver;
    QTimer *timer;
    uint8_t loggedState;
    uint8_t vehicleID;
    uint8_t armedState;
    uint8_t systemMode;
    uint8_t gps_fix;
    uint8_t failsafe_flags;
    uint8_t system_health;
    uint16_t sensors_validity;
    uint32_t uptime_ms_heartbeat;
    VEDTP_Main vedtp;
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    uint32_t uptime_ms_ahrs;
    float temperature_C;         // Air/Water temp
    float pressure_mbar;          // Barometric or depth-based
    float humidity_RH;           // Only valid in air/land/surface
    float altitude_m;            // Derived from pressure (air/land only)
    float tvoc_ppb;              // Total VOCs
    float eco2_ppm;              // CO₂ equivalent
    uint16_t air_quality_index;  // AQI score, optional
    float depth_m;               // From pressure sensor (e.g., MS5837)
    float salinity_ppt;          // Parts per thousand (marine salinity)
    float conductivity_uS;       // Optional: more accurate than salinity
    float water_dissolved_oxygen_mgL;  // Optional for aquatic monitoring
    float turbidity_NTU;// Optional: underwater vision/dirtiness
    float range_m;
    float confidence;
    uint8_t device_id;   // Which device (AHRS, GPS, etc.)
    uint8_t error_type;  // Type of fault (init, data, timeout, etc.)
    uint8_t severity;    // 0 = Info, 1 = Warning, 2 = Critical
    uint8_t flags;       // Bitfield: persistent, auto-reset, etc.
    uint32_t uptime_ms_error;
    uint8_t  command;        // Command ID this ACK refers to
    uint8_t  result;         // See ResultCode enum below
    uint8_t  source;         // Who responded (e.g. FC, NAV, CAM)
    uint8_t  flags_ack;          // Optional: retry, delayed, etc.
    QString  message;    // Optional: "OK", "Param error", "Timed out"
    uint32_t uptime_ms_ack;
    uint8_t sensor_status_flags; // Bitfield: sensor errors, low range, etc.
    uint8_t medium_type;
    float velocity[4];      // 8B: Linear speed (m/s)
    uint8_t decode_status;
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
    //void sendMode(System_Mode mode);
    uint uptime_ms;
    HEARTBEAT_Packet heartbeat_data;
    AHRS_Packet ahrs_data;
    EXTERNAL_ATMOSPHERE_Packet External_atmosphere_data;
    POWER_HEALTH_Packet Power_health_data;
    SONAR_Packet sonar_data;
    MOTOR_Packet motor_data;
    DEVICE_ERROR_Packet error_data;
    //COMMAND_Packet ;
    COMMAND_ACK_Packet acknowledgement_data;
    QTimer randomTimer;
    QElapsedTimer yawTimer;
    QString log_path="";
    QString pdf_path="";
    QString video_path="";
    QString error_path="";
    int y_axis_left = 1500, x_axis_left = 1500, x_axis_right = 1500, y_axis_right=1500;
    int light_intensity =1500;
    int mode_manual = 0, mode_depth=0, mode_stab = 0, mod_custom =0;
    QTimer *joystickTimer;
    QTimer *heartbeatTimer;
    bool is_depth_hold = false;
    JoystickController joystickcontroller;
    double value;
    int outputMin=1100, outputMax=1900, InputMin=-1, InputMax=1;

};
#endif
