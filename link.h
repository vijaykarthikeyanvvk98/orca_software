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
#include <qdir.h>
#include <qmutex.h>
#include "VEDTP.h"
#include "qstandardpaths.h"
#include <QQueue>      // Required for QQueue
#include <QVariant>
#include <QVector>
#include <QtMath>// For qSin, qCos, qAsin, qSqrt, etc.
#include <qprocess.h>
class Link:public QObject
{
    Q_OBJECT
    Q_PROPERTY(float yaw_deg READ yaw_value WRITE setYaw_value NOTIFY yawValueChanged)
    Q_PROPERTY(float pitch_deg READ pitch_value WRITE setPitch_value NOTIFY pitchValueChanged)
    Q_PROPERTY(float roll_deg READ roll_value WRITE setRoll_value NOTIFY rollValueChanged)
    Q_PROPERTY(quint64 uptime_ms_ahrs READ uptime_ms_ahrs_value WRITE setuptime_ms_ahrs_value NOTIFY uptimeChanged)
    Q_PROPERTY(float temperature_C READ temperature WRITE settemperature NOTIFY temperatureChanged)
    Q_PROPERTY(float depth_m READ depth WRITE setdepth NOTIFY depthChanged)
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

    Q_PROPERTY(float temperature_imuC READ imu_temperature WRITE set_imu_temp NOTIFY imu_temp_changed)
    Q_PROPERTY(uint8_t systemcalibration READ imu_system WRITE set_imu_uptime NOTIFY imu_uptime_changed)
    Q_PROPERTY(uint8_t gyrocalibration READ imu_accelerometer WRITE set_imu_magnetometer NOTIFY imu_magnetometer_changed)
    Q_PROPERTY(uint8_t accelerometercalibration READ imu_gyro WRITE set_imu_accelerometer NOTIFY imu_gyro_changed)
    Q_PROPERTY(uint8_t magnetometercalibration READ imu_magnetometer WRITE set_imu_gyro NOTIFY imu_accelerometer_changed)
    Q_PROPERTY(uint32_t uptime_ms_imu READ yaw_value WRITE set_imu_system NOTIFY imu_system_changed)


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

    float imu_temperature() const;
    uint8_t imu_system() const;
    uint8_t imu_accelerometer() const;
    uint8_t imu_gyro() const;
    uint8_t imu_magnetometer() const;
    uint32_t imu_uptime() const;
    /*float temperature() const;
    float temperature() const;
    float temperature() const;
    float temperature() const;
    float temperature() const;
    float temperature() const;*/

signals:
    void errordetected();
    void set_heartbeat_data();
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
    void joystick_changed(bool);
    void vehicle_ad_status(int);
    void vehicle_mod_status(int);
    void gain_status(int);
    void disable_depth_mode();
    void enable_depth_mode();
    void stop_timer_blinking();

    void imu_temp_changed();
    void imu_uptime_changed();
    void imu_magnetometer_changed();
    void imu_gyro_changed();
    void imu_accelerometer_changed();
    void imu_system_changed();
    void imu_updated();
    //void sendBytes(const QByteArray&);
public slots:
    void save_short();
    void update_heartbeat();
    void set_mode(int);
    void gain_update(int);
    void joystick_controller_updated(QString,bool);
    void arm_disarm(int,bool);
    void send_arm(ARM_State);
    void login(QString username, QString password);
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
    void imu_parsing();
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
    QVariantList errorcatched();
    QVariantList imu_data();
    void set_imu_temp(float);
    void set_imu_uptime(uint32_t);
    void set_imu_magnetometer(uint8_t);
    void set_imu_accelerometer(uint8_t);
    void set_imu_gyro(uint8_t);
    void set_imu_system(uint8_t);
    void check_env();
private:
    QVariantList errorArray;
    Receiver receiver;
    QTimer *timer=nullptr;
    uint8_t loggedState=0;
    uint8_t vehicleID=VEHICLE_ROV6;
    uint8_t armedState=0,armedState_prev=0;
    uint8_t systemMode=0,systemMode_prev=0;
    uint8_t gps_fix=0;
    uint8_t failsafe_flags=0;
    uint8_t system_health=0;
    uint16_t sensors_validity=0;
    uint32_t uptime_ms_heartbeat=0;
    VEDTP_Main vedtp;
    float yaw_deg=0.0;
    float pitch_deg=0.0;
    float roll_deg=0.0;
    uint32_t uptime_ms_ahrs=0;
    float temperature_C=0.0;         // Air/Water temp
    float pressure_mbar=0.0;          // Barometric or depth-based
    float humidity_RH=0.0;           // Only valid in air/land/surface
    float altitude_m=0.0;            // Derived from pressure (air/land only)
    float tvoc_ppb=0.0;              // Total VOCs
    float eco2_ppm=0.0;              // CO₂ equivalent
    uint16_t air_quality_index=0;  // AQI score, optional
    float depth_m=0.0;               // From pressure sensor (e.g., MS5837)
    float salinity_ppt=0.0;          // Parts per thousand (marine salinity)
    float conductivity_uS=0.0;       // Optional: more accurate than salinity
    float water_dissolved_oxygen_mgL=0.0;  // Optional for aquatic monitoring
    float turbidity_NTU=0.0;// Optional: underwater vision/dirtiness
    float range_m=0.0;
    float confidence=0.0;
    uint8_t device_id=-1;   // Which device (AHRS, GPS, etc.)
    uint8_t error_type=0;  // Type of fault (init, data, timeout, etc.)
    uint8_t severity=0;    // 0 = Info, 1 = Warning, 2 = Critical
    uint8_t flags=0;       // Bitfield: persistent, auto-reset, etc.
    uint32_t uptime_ms_error=0;
    uint8_t  command=0;        // Command ID this ACK refers to
    uint8_t  result=0;         // See ResultCode enum below
    uint8_t  source=0;         // Who responded (e.g. FC, NAV, CAM)
    uint8_t  flags_ack=0;          // Optional: retry, delayed, etc.
    QString  message=0;    // Optional: "OK", "Param error", "Timed out"
    uint32_t uptime_ms_ack=0;
    uint8_t sensor_status_flags=0; // Bitfield: sensor errors, low range, etc.
    uint8_t medium_type=0;
    float velocity[4];      // 8B: Linear speed (m/s)
    uint8_t decode_status=0;
    float smps_voltage=0.0;        // Output of SMPS
    float battery_voltage=0.0;     // Battery pack voltage
    float current_draw=0.0;        // Total current drawn from battery or SMPS (A)
    float battery_capacity_pct=0.0;// 0.0–100.0% (if fuel gauge or estimated)
    float power_usage_watt=0.0;    // Power = voltage * current (W)
    float estimated_uptime_min=0.0;// Time remaining if discharging (minutes)
    uint8_t charging=0;          // 1 = charging, 0 = not
    uint8_t discharging=0;       // 1 = active draw from battery
    uint8_t battery_present=0;   // 1 = battery connected
    uint8_t power_flags=0;       // Bitfield: overvolt, undervolt, fail, etc.
    //void sendMode(System_Mode mode);
    uint uptime_ms=0;
    HEARTBEAT_Packet heartbeat_data,heartbeat_data2;
    AHRS_Packet ahrs_data;
    EXTERNAL_ATMOSPHERE_Packet External_atmosphere_data;
    POWER_HEALTH_Packet Power_health_data;
    SONAR_Packet sonar_data;
    MOTOR_Packet motor_data;
    DEVICE_ERROR_Packet error_data;
    IMU_Packet IMU_data;
    //COMMAND_Packet ;
    COMMAND_ACK_Packet acknowledgement_data;
    ARM_State arm_status;
    QTimer randomTimer;
    QElapsedTimer yawTimer;
    QString video_path="";
    QString error_path="";
    int y_axis_left = 1500, x_axis_left = 1500, x_axis_right = 1500, y_axis_right=1500;
    int light_intensity =1500;
    int mode_manual = 0, mode_depth=0, mode_stab = 0, mod_custom =0;
    QTimer *joystickTimer=nullptr;
    QTimer *heartbeatTimer=nullptr;
    bool is_depth_hold = false;
    JoystickController joystickcontroller;
    double value;
    int outputMin=1100, outputMax=1900, InputMin=-1, InputMax=1;
    QString username ="";
    QString password ="";
    float gain_plus=0.25,gain_minus=1;
    int delta_gain=0;
    std::atomic<bool> writing_done{true};  // Atomic flag

    QFile File,File2,File3,File4,File5,File6,File7,File8,File9,File10,File11;
    QTextStream stream,stream2,stream3,stream4,stream5,stream6,stream7,stream8,stream9,stream10,stream11;
    int array_size=0,array_size2=0,array_size3=0,array_size4=0,array_size5=0;
    QMutex sendMutex,bufferMutex,parseMutex,mutex1,mutex11,mutex2,mutex20,mutex21,mutex22,mutex23,mutex24,mutex25,mutex26,mutex12,mutex3,mutex13,mutex4,mutex14,mutex15,mutex5,mutex6,mutex16,mutex7,mutex17,mutex8,mutex18,mutex9,mutex19,mutex10,health_mutex,streamMutex,file_mutex,streamMutex2,file_mutex2,streamMutex3,file_mutex3,streamMutex4,file_mutex4,streamMutex5,file_mutex5,mutex;
    QDateTime date,date2;
    QString formattedTime, formattedTime2,formattedTime3,formattedTime4;
    QByteArray formattedTimeMsg,formattedTimeMsg2,formattedTimeMsg3;

    QString from = "", to = "", selected_time = "";
    QVector<QString> twoDArray, twoDArray2, twoDArray3, twoDArray_text,errDArray,bbDArray;

    // Modify the target directory to add a subdirectory for your files.
    QString logFileDir5="";
    QDateTime currentDateTime,currentDateTime2;
    QString logFilePath="",csvFilePath="";
    bool start_once=true;
    bool ahrs_once=true;
    bool gps_once=true;
    bool imu_once=true;
    bool env_once=true;
    bool sys_once=true;
    bool leak_once=true;
    bool flow_once=true;
    bool enc_once=true;
    bool cpt_once=true;
    bool sonar_once=true;
    bool err_once=true;
    bool ack_once=true;
    bool heart_once=true;
    bool joy_once=false;

    //converted Image Path
    QString map_image_path = "";
    QString log_path = "";
    QString pdf_path="";
    QString tilesDir="";
    QString default_graph_path = "";
    QString err_path = "";
    QString err_path2 = "";
    QString err_path3 = "";
    QString err_path4 = "";
    QString err_path5 = "";
    QString bb_path = "";
    QString bb_path2 = "";

    //Python processing
    QProcess process;
    QString graph_path="",graph_path2="";
    QString cpt_path="",cpt_path2="";


    QString error_device="",err_type="";
    //Logfiles
    QString logFileDir4 = "";
    QString element="",element2="",element3="",title_element="Date,Time,,Device Id,Log Status,Vehicle\n",graph_element="",graph_element2="";
    int array_count=0,temp_i=0;
    QByteArray temp_array;
    qint64 startTime = 0, elapse_time = 0;
    qint64 latency, prev_latency = 0;
    QElapsedTimer timer2;
    QTimer *timer3=nullptr;
    QVariantList array, array2, array_3, health_check_array,stream_Array,err_array;
    QVariantList heart_array,ahrs_Array,gps_array,imu_array,env_array,sys_array,sonar_array,flow_array,leak_array,cpt_array,graph_array,encoder_array,dead_array,error_array,joy_array,cpt_1m;


    //IMU
    float acceleration[3];
    float gyro[3];
    float magnetic[3];
    float rotationvector[3];
    float linearacceleration[3];
    float gravity[3];
    float temperature_imuC=0.0;
    uint8_t systemcalibration=0;
    uint8_t gyrocalibration=0;
    uint8_t accelerometercalibration=0;
    uint8_t magnetometercalibration=0;
    uint32_t uptime_ms_imu=0;

    QTimer *watchdog = nullptr;
    float temperature_C_prev=0.0;         // Air/Water temp
    float depth_m_prev=0.0;         // Air/Water temp
    float pressure_mbar_prev=0.0;         // Air/Water temp

};
#endif
