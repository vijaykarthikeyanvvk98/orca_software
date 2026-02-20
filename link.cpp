#include "link.h"
#include <QDebug>
#include <QObject>
#include <QDateTime>
#include <QDir>
#include <cmath>
#include <QRandomGenerator>
#include <QStandardPaths>
#include "VEDTP.h"
#include <QMutexLocker>
#include <iostream>
#include <QtConcurrent/QtConcurrent>  // For QtConcurrent::run()
#include "videostreamer.h"
VideoStreamer videostream;

Link::Link()
{
    joystickcontroller.initialize();
    connect(this, &Link::_invokeWriteBytes, this, &Link::send_data);
    connect(&receiver,&Receiver::datareceived,this, &Link::send_data_to_process,Qt::QueuedConnection);
    connect(this, &Link::data_processed, this,&Link::data_to_be_updated);
    /*connect(&randomTimer, &QTimer::timeout,
            this, &Link::generateRandomData);*/
    heartbeatTimer = new QTimer(this);
    connect(heartbeatTimer, &QTimer::timeout, this, &Link::send_heart_beat);
    heartbeatTimer->start(Freq(FREQ_HEARTBEAT));
    //send_heart_beat();
    joystickTimer = new QTimer(this);
    connect(joystickTimer, &QTimer::timeout, this, &Link::sendJoystickData);
    joystickTimer->start(Freq(FREQ_JOYSTICK));
    connect(&joystickcontroller, &JoystickController::leftaxisX,this,&Link::set_joystick_values);
    connect(&joystickcontroller, &JoystickController::leftaxisY,this,&Link::set_joystick_values2);
    connect(&joystickcontroller, &JoystickController::rightaxisX,this,&Link::set_joystick_values3);
    connect(&joystickcontroller, &JoystickController::rightaxisY,this,&Link::set_joystick_values4);
    connect(&joystickcontroller, &JoystickController::arm_status,this,&Link::arm_disarm);
    connect(&joystickcontroller, &JoystickController::mode,this,&Link::set_mode);
    connect(&joystickcontroller, &JoystickController::joystick_detected,this,&Link::joystick_controller_updated);
    connect(&joystickcontroller, &JoystickController::set_gain,this,&Link::gain_update);
    create_directory();

    //emit vehicle_ad_status(1);
}
Link::~Link()
{
    if(heartbeatTimer && heartbeatTimer->isActive())
        heartbeatTimer->stop();
    if(joystickTimer && heartbeatTimer->isActive())
        joystickTimer->stop();

    heartbeatTimer=nullptr;
    joystickTimer=nullptr;

    temp_array.clear();
    formattedTimeMsg.clear();
    formattedTimeMsg2.clear();
    formattedTimeMsg3.clear();
    array.clear();
    array2.clear();
    array_3.clear();
    health_check_array.clear();
    stream_Array.clear();
    err_array.clear();
    heart_array.clear();
    ahrs_Array.clear();
    gps_array.clear();
    imu_array.clear();
    env_array.clear();
    sys_array.clear();
    sonar_array.clear();
    flow_array.clear();
    leak_array.clear();
    cpt_array.clear();
    graph_array.clear();
    encoder_array.clear();
    dead_array.clear();
    error_array.clear();
    joy_array.clear();


}
void Link::send_heart_beat()
{
    heartbeat_data2.loggedState        = loggedState;       // System logged state      ( enum : Logged_State)
    heartbeat_data2.vehicleID          = vehicleID;      // Vehicle type             ( enum : Vehicle_ID)
    heartbeat_data2.armedState         = armedState;         // System armed state       ( enum : ARM_State)
    heartbeat_data2.systemMode         = systemMode;        // System mode state        (enum : System_Mode)
    heartbeat_data2.gps_fix            = gps_fix;       // System gps fix           (enum : GPSFixStatus)
    heartbeat_data2.failsafe_flags     = failsafe_flags;    // System failsafe flags    (enum : Failsafe_flags)
    heartbeat_data2.system_health      = system_health; // System health flag       (enum : SystemHealth)
    heartbeat_data2.sensors_validity   = sensors_validity;
    heartbeat_data2.uptime_ms =static_cast<uint32_t>(QDateTime::currentDateTime().toMSecsSinceEpoch());
    VEDTP_Main heartbeat_send_packet;                                                   // for holding the main vedtp packet

    uint8_t encode_status = encode_HEARTBEAT(&heartbeat_send_packet,&heartbeat_data2);   // encode heartbeat function from vedtp protocol library
    //qDebug()<<heartbeat_send_packet.device;
    if(encode_status)
    {
    //mavlink_send_buffer(heartbeat_send_packet);
        emit _invokeWriteBytes(
            QByteArray(reinterpret_cast<char*>(&heartbeat_send_packet),
                       sizeof(VEDTP_Main))
            );
    }

    heartbeat_data2={};
}
void Link::mavlink_send_buffer(VEDTP_Main vedtp_send)
{
    //VEDTP_Main vedtp_send;
    //QMutexLocker locker(&mutex1);
    /*constexpr*/const size_t bufferSize = sizeof(vedtp_send);
    uint8_t buffer[bufferSize];
    memcpy(buffer, &vedtp_send, bufferSize);
    writeByteThreadSafe(reinterpret_cast<const char*>(buffer), bufferSize);

}
void Link::writeByteThreadSafe(const char *bytes, int length)
{
    emit _invokeWriteBytes(QByteArray(bytes, length));
}
void Link::send_data(QByteArray buffer)
{
    if (buffer.isEmpty()) {
        qWarning() << "Empty buffer ignored";
        return;
    }
    bool invoked = QMetaObject::invokeMethod(&receiver, [this, buffer]() {
        receiver.senddatagram(buffer);
    },Qt::QueuedConnection);

    if (!invoked) {
        qCritical() << "Failed to queue datagram";
    }
    //receiver.senddatagram(buffer);

}

void Link::send_data_to_process(QByteArray data)
{
    //qDebug()<<data.data();
    static VEDTP_Main vedtp_recv; // declare vedtp_recv
    uint8_t data_buffer_storage[VEDTP_HEADER_SIZE];
    for(uint16_t i = 0; i < data.size(); i++){
       uint8_t Vedtp_feed_status = feed_Me_Bytes(data[i], &vedtp_recv);
        switch (Vedtp_feed_status){                                                                // check the status for feeding bytes
        case VEDTP_INCOMPLETE:                                                      // packet is not fully arrived yes, just wait and do nothing
            break;
        case VEDTP_COMPLETE:                                                        // got the complete vedtp packet
            //qDebug()<<"DEVICEID"<<vedtp_recv.device;
            switch (vedtp_recv.device){                                             // check the device id
                case DEVICE_HEARTBEAT:                                              // if the device id matches the heartbeat device id,
               // qDebug()<<"Packet has Heartbeat Data.\n";
                    heartbeat_data={};

                decode_status = decode_HEARTBEAT(&vedtp_recv, &heartbeat_data);         // decode the heartbeat packet
                if(decode_status){                                                              // if decode is success,
                    //qDebug()<<"Heartbeat data decoded successfully.\n\n";                         // print or process the data
                    /*qDebug()<<"Logged:"<<heartbeat_data.loggedState<<
                        "ID:"<<heartbeat_data.vehicleID<<
                        "ARM:"<<heartbeat_data.armedState<<
                        "Mode:"<<heartbeat_data.systemMode<<
                        "GPS Fix:"<<heartbeat_data.gps_fix<<
                        "Failsafe:"<<heartbeat_data.failsafe_flags<<
                        "health:"<<heartbeat_data.system_health<<
                        "validity:"<<heartbeat_data.sensors_validity<<
                        "uptime:"<<heartbeat_data.uptime_ms;*/
                    emit data_processed(DEVICE_HEARTBEAT);
                }
                else {
                    //qDebug()<<"Heartbeat data decoding failed, Check the parameters.\n";
                }
                break;
                //gps
                case DEVICE_AHRS:
                    //qDebug()<<"Packet has AHRS Data.\n";
                    ahrs_data={};
                    decode_status = decode_AHRS(&vedtp_recv, &ahrs_data);         // decode the heartbeat packet
                if(decode_status){                                                              // if decode is success,
                    /*qDebug()<<"AHRS data decoded successfully.\n\n";                         // print or process the data
                    qDebug()<<ahrs_data.yaw_deg<<
                        ahrs_data.pitch_deg<<
                        ahrs_data.roll_deg<<
                        ahrs_data.uptime_ms;*/
                    emit data_processed(DEVICE_AHRS);
                }
                else {
                    //qDebug()<<"AHRS data decoding failed, Check the parameters.\n";
                }
                break;
                //imu
                case DEVICE_IMU:
                    //qDebug()<<"Packet has IMU data";
                    IMU_data={};
                    decode_status = decode_IMU(&vedtp_recv, &IMU_data);
                    if(decode_status){
                        /*qDebug()<<"IMU data decoded successfully";
                        qDebug()<<IMU_data.acceleration[0]
                                 <<IMU_data.gyro[0]
                                 <<IMU_data.magnetic[0]
                                 <<IMU_data.rotationvector[0]
                                 <<IMU_data.linearacceleration[0]
                                 <<IMU_data.gravity[0]
                                 <<IMU_data.temperature_C
                                 <<IMU_data.systemcalibration
                                 <<IMU_data.gyrocalibration
                                 <<IMU_data.accelerometercalibration
                                 <<IMU_data.magnetometercalibration
                                 <<IMU_data.uptime_ms;*/
                                    emit data_processed(DEVICE_IMU);
                    }
                    else{
                        //qDebug()<<"IMU data decoding failure";
                    }
                    break;
                case DEVICE_EXTERNAL_ATMOSPHERE:                                              // if the device id matches the heartbeat device id,
                    //qDebug()<<"Packet has EXTERNAL ATMOSPHERE DATA.\n";
                    External_atmosphere_data={};
                    decode_status = decode_EXTERNAL_ATMOSPHERE(&vedtp_recv, &External_atmosphere_data);         // decode the heartbeat packet
                    if(decode_status){                                                              // if decode is success,
                        //qDebug()<<"External data decoded successfully.\n\n";                         // print or process the data
                        //qDebug()<<External_atmosphere_data.temperature_C<<External_atmosphere_data.pressure_mbar<<

                            // Air/Water temp
                        /*External_atmosphere_data.depth_m<<               // From pressure sensor (e.g., MS5837)
                        External_atmosphere_data.salinity_ppt;  */        // Parts per thousand (marine salinity)
                        emit data_processed(DEVICE_EXTERNAL_ATMOSPHERE);
                    }
                    else {
                        //qDebug()<<"external atmosphere data decoding failed, Check the parameters.\n";
                     }
                break;
                case DEVICE_POWER_HEALTH:                                              // if the device id matches the heartbeat device id,
                    //qDebug()<<"Packet has Power Health DATA.\n";
                    Power_health_data={};
                    decode_status = decode_POWER_HEALTH(&vedtp_recv, &Power_health_data);         // decode the heartbeat packet
                    if(decode_status){                                                              // if decode is success,
                        //qDebug()<<"Power Health data decoded successfully.\n\n";                         // print or process the data
                        //qDebug()<<Power_health_data.battery_voltage;          // Parts per thousand (marine salinity)
                        emit data_processed(DEVICE_POWER_HEALTH);
                    }
                    else {
                        //qDebug()<<"power health data decoding failed, Check the parameters.\n";
                    }
                break;
                case DEVICE_SONAR:                                              // if the device id matches the heartbeat device id,
                    //qDebug()<<"Packet has Sonar Data.\n";
                    sonar_data={};
                    decode_status = decode_SONAR(&vedtp_recv, &sonar_data);         // decode the heartbeat packet
                    if(decode_status){                                                              // if decode is success,
                        /*qDebug()<<"Sonar data decoded successfully.\n\n";                         // print or process the data
                        qDebug()<<sonar_data.range_m<<
                                sonar_data.confidence;*/
                        emit data_processed(DEVICE_SONAR);
                    }
                    else {
                        qDebug()<<"Sonar data decoding failed, Check the parameters.\n";
                    }
                break;
        case DEVICE_MOTOR:                                              // if the device id matches the heartbeat device id,
                //qDebug()<<"Packet has Motor Data.\n";
                motor_data={};
                decode_status = decode_MOTOR(&vedtp_recv, &motor_data);         // decode the heartbeat packet
                if(decode_status){                                                              // if decode is success,
                    /*qDebug()<<"Device motor data decoded successfully.\n\n";                         // print or process the data
                    qDebug()<<motor_data.velocity[3];*/
                       // sonar_data.confidence;
                    emit data_processed(DEVICE_MOTOR);
                }
                else {
                   // qDebug()<<"Motor data decoding failed, Check the parameters.\n";
                }
                break;
                case DEVICE_DEVICE_ERROR:
                error_data={};                                // if the device id matches the heartbeat device id,
                decode_status = decode_DEVICE_ERROR(&vedtp_recv, &error_data);         // decode the heartbeat packet
                if(decode_status){                                                              // if decode is success,
                    //qDebug()<<"Device error decoded successfully.\n\n";                         // print or process the data
                    /*qDebug()<<"Error data :"<<error_data.device_id<<error_data.error_type<<
                        error_data.severity<<error_data.flags<<
                        error_data.uptime_ms;*/
                    emit data_processed(DEVICE_DEVICE_ERROR);
                }
                else {
                    //qDebug()<<"Error data decoding failed, Check the parameters.\n";
                }
                break;
                case DEVICE_COMMAND_ACK:                                              // if the device id matches the heartbeat device id,
                //qDebug()<<"Packet has Command Acknowledgement Data.\n";
                    acknowledgement_data={};
                decode_status = decode_COMMAND_ACK(&vedtp_recv, &acknowledgement_data);         // decode the heartbeat packet
                if(decode_status){                                                              // if decode is success,
                    /*qDebug()<<"command acknowldgement data decoded successfully.\n\n";                         // print or process the data
                    qDebug()<<
                        acknowledgement_data.command<<
                        acknowledgement_data.result<<
                        acknowledgement_data.source<<
                        acknowledgement_data.flags<<
                        acknowledgement_data.message<<
                        acknowledgement_data.uptime_ms;*/
                    emit data_processed(DEVICE_COMMAND_ACK);
                }
                else {
                    //qDebug()<<"Command Acknowledgement data decoding failed, Check the parameters.\n";
                }
                break;
            }
                break;
                case VEDTP_INVALID_START_BYTES:                                             // invalid start byte, means packet is not started yet, just do nothing

                break;
                case VEDTP_INVALID_SECOND_BYTES:                                            // invalid second byte, means either the first byte triggered with random data or packet got corrupted. do nothing or show error
                    qDebug()<<"Invalid Senond Byte";
                break;
                case VEDTP_CRC_FAIL:                                                        // crc checking fail, packet is corrupted. do nothing or show error
                    qDebug()<<"CRC Fail";
                break;
                case VEDTP_PROTOCOL_VERSION_MISMATCH:                                       // Protocol version mismatch. show error or update protocol version
                    qDebug()<<"Protocol version mismatch";
                break;
                default:
                break;
        }
    }
}
void Link::data_to_be_updated(int device)
{
    switch(device){
    case DEVICE_HEARTBEAT:
        heart_beat_parsing();
        break;
    case DEVICE_AHRS:
        AHRS_parsing();
        break;
    case DEVICE_EXTERNAL_ATMOSPHERE:
        env_parsing();
        break;
    case DEVICE_SONAR:
        sonar_parsing();
        break;
    case DEVICE_MOTOR:
        motor_parsing();
        break;
    case DEVICE_COMMAND_ACK:
        command_parsing();
        break;
    case DEVICE_DEVICE_ERROR:
       // qDebug()<<"error_parsing";
        error_parsing();
        break;
    }
}
void Link::heart_beat_parsing()
{
    loggedState = heartbeat_data.loggedState;
    vehicleID = heartbeat_data.vehicleID;
    armedState = heartbeat_data.armedState;
    systemMode = heartbeat_data.systemMode;
    gps_fix = heartbeat_data.gps_fix;
    failsafe_flags = heartbeat_data.failsafe_flags;
    system_health= heartbeat_data.system_health;
    sensors_validity = heartbeat_data.sensors_validity;
    uptime_ms_heartbeat = heartbeat_data.uptime_ms;
    switch(loggedState)
    {
    case 0:
        login("VOTPL","VOTPL");
        break;
    case 1:
        if(heart_once)
        {
            stream.flush();
            stream2.flush();
            File.close();
            File2.close();
            // Construct the file paths relative to the target directory.
            //QString logFilePath = logFileDir + "/logfile-"+ currentDateTime.toString("dd.MM.yyyy") +".txt";
            currentDateTime = QDateTime::currentDateTime();
            formattedTime2 = currentDateTime.toString("dd.MM.yyyy-hh.mm.ss");
            logFilePath = logFileDir5 + "/logfile-" + QString(formattedTime2) + ".txt";
            csvFilePath= logFileDir5 + "/logfile-" + QString(formattedTime2) + ".csv";

            File.setFileName(logFilePath);
            File2.setFileName(csvFilePath);

            //qDebug()<<File.fileName();
            if (File.open(QIODevice::Append | QIODevice::ReadWrite)
                && File2.open(QIODevice::Append | QIODevice::ReadWrite)) {
                stream.setDevice(&File);
                stream2.setDevice(&File2);
                //stream2 <<log_heading;
            }
            heart_once=false;
            joy_once=true;
        }

        break;
    default:
        break;
    }
    if(armedState != armedState_prev)
        emit vehicle_ad_status(armedState);
    if(systemMode != systemMode_prev)
        emit vehicle_mod_status(systemMode);

if(File.isOpen())
{
    QMutexLocker locker(&mutex15);
    formattedTimeMsg.clear();
    date = QDateTime::currentDateTime();
    formattedTime = date.toString("dd.MM.yyyy,hh:mm:ss");
    formattedTimeMsg = formattedTime.toLocal8Bit();
    twoDArray.append(QString(formattedTimeMsg) + "," +QString::number(DEVICE_HEARTBEAT) +","+ QString::number(loggedState) +","+ QString::number(vehicleID)+","+ QString::number(armedState)+","+ QString::number(systemMode)+","+ QString::number(gps_fix)+","+ QString::number(failsafe_flags)+","+ QString::number(system_health)+","+ QString::number(sensors_validity)+","+ QString::number(uptime_ms_heartbeat)+"\n\n");
    array_size++;
    writing_done=false;
    save_short();
}
else;

    armedState_prev = armedState;
    systemMode_prev = systemMode;
}
void Link::AHRS_parsing()
{
   // if (yawTimer.elapsed() >= 1000) {
    //ahrs_data.yaw_deg +=90;
   try
   {
   ahrs_data.yaw_deg = std::fmod(ahrs_data.yaw_deg, 360.0f);;
        setYaw_value(ahrs_data.yaw_deg);
       // yawTimer.restart();
    //}
    //yaw_deg = ahrs_data.yaw_deg;
        setPitch_value(ahrs_data.pitch_deg);
        setRoll_value(ahrs_data.roll_deg);
        setuptime_ms_ahrs_value(ahrs_data.uptime_ms);

        if (qIsNaN(ahrs_data.yaw_deg )) {
            throw std::runtime_error("NaN detected in yaw");
        }
        if (qIsNaN(ahrs_data.pitch_deg)) {
            throw std::runtime_error("NaN detected in pitch");
        }
        if (qIsNaN(ahrs_data.roll_deg)) {
            throw std::runtime_error("NaN detected in roll");
        }

        if (ahrs_data.yaw_deg  == std::numeric_limits<double>::infinity() || ahrs_data.yaw_deg  == -std::numeric_limits<double>::infinity()) {
            throw std::overflow_error("Floating point overflow occurred.");
        }

        if (ahrs_data.pitch_deg == std::numeric_limits<double>::infinity() || ahrs_data.pitch_deg == -std::numeric_limits<double>::infinity()) {
            throw std::overflow_error("Floating point overflow occurred.");
        }

        if (ahrs_data.roll_deg == std::numeric_limits<double>::infinity() || ahrs_data.roll_deg == -std::numeric_limits<double>::infinity()) {
            throw std::overflow_error("Floating point overflow occurred.");
        }

   }
   catch (const std::out_of_range& e) {
       std::cerr << "Index out of range error:" << e.what() << std::endl;
   }
   catch (const std::overflow_error& e) {
       std::cerr << "Caught overflow: " << e.what() << std::endl;
   }
   catch (const std::invalid_argument& e) {
       std::cerr << "Invalid argument:" << e.what() << std::endl;
   }
   catch (const std::logic_error& e) {
       std::cerr << "Logic error:" << e.what() << std::endl;
   }
   catch (const std::runtime_error& e) {
       std::cerr << "Runtime error:" << e.what() << std::endl;
   }
   catch (const std::exception& e) {
       std::cerr<<"Error:"<<&e << std::endl;
   }
   catch(...)
   {
       std::cerr<<"Error Unkown" << std::endl;

   }

   ahrs_Array.clear();
   ahrs_Array.append(ahrs_data.yaw_deg);
   ahrs_Array.append(ahrs_data.pitch_deg);
   ahrs_Array.append(ahrs_data.roll_deg);
   if(File.isOpen())
   {
       QMutexLocker locker(&mutex16);
       //twoDArray.append(title_element3);
       formattedTimeMsg.clear();
       date = QDateTime::currentDateTime();
       formattedTime = date.toString("dd.MM.yyyy,hh:mm:ss");
       formattedTimeMsg = formattedTime.toLocal8Bit();
       twoDArray.append(QString(formattedTimeMsg) + "," +QString::number(DEVICE_AHRS) +","+ QString::number(ahrs_data.yaw_deg) +","+ QString::number(ahrs_data.pitch_deg)+","+QString::number(ahrs_data.roll_deg)+"\n\n");
       array_size++;
       writing_done=false;
       save_short();
   }
   videostream.set_ahrs(ahrs_Array);

}
void Link::env_parsing()
{
    try
    {
    settemperature(External_atmosphere_data.temperature_C);
    setdepth(External_atmosphere_data.depth_m);
    setsalinity(External_atmosphere_data.salinity_ppt);
    if (qIsNaN(External_atmosphere_data.depth_m)) {
        throw std::runtime_error("NaN detected in Env Depth");
    }
    if (qIsNaN(External_atmosphere_data.pressure_mbar)) {
        throw std::runtime_error("NaN detected in Env Pressure");
    }
    if (qIsNaN(External_atmosphere_data.temperature_C)) {
        throw std::runtime_error("NaN detected in Env Temperature");
    }

    if (External_atmosphere_data.depth_m == std::numeric_limits<double>::infinity() || External_atmosphere_data.depth_m == -std::numeric_limits<double>::infinity()) {
        throw std::overflow_error("Floating point overflow occurred.");
    }

    if (External_atmosphere_data.pressure_mbar == std::numeric_limits<double>::infinity() || External_atmosphere_data.pressure_mbar == -std::numeric_limits<double>::infinity()) {
        throw std::overflow_error("Floating point overflow occurred.");
    }

    if (External_atmosphere_data.temperature_C == std::numeric_limits<double>::infinity() || External_atmosphere_data.temperature_C == -std::numeric_limits<double>::infinity()) {
        throw std::overflow_error("Floating point overflow occurred.");
    }

    }
    catch (const std::out_of_range& e) {
        std::cerr << "Index out of range error:" << e.what()<< std::endl;
    }
    catch (const std::overflow_error& e) {
        std::cerr << "Caught overflow: " << e.what() << std::endl;
    }
    catch (const std::invalid_argument& e) {
        std::cerr << "Invalid argument:" << e.what() << std::endl;
    }
    catch (const std::logic_error& e) {
        std::cerr << "Logic error:" << e.what() << std::endl;
    }
    catch (const std::runtime_error& e) {
        std::cerr << "Runtime error:" << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr<<"Error:"<<&e << std::endl;
    }
    catch(...)
    {
        std::cerr<<"Error Unkown" << std::endl;

    }

    if(File.isOpen())
    {
        formattedTimeMsg.clear();
        date = QDateTime::currentDateTime();
        formattedTime = date.toString("dd.MM.yyyy,hh:mm:ss");
        formattedTimeMsg = formattedTime.toLocal8Bit();
        twoDArray.append(QString(formattedTimeMsg) + "," +QString::number(DEVICE_EXTERNAL_ATMOSPHERE) +","+ QString::number(External_atmosphere_data.depth_m) +","+ QString::number(External_atmosphere_data.pressure_mbar)+","+QString::number(External_atmosphere_data.temperature_C)+"\n\n");
        array_size++;
        writing_done=false;
        save_short();
    }
    else;

}
void Link::error_parsing()
{
    // setDeviceId(error_data.device_id);
    // setErrorType( error_data.error_type);
    // setseverity(error_data.severity);
    // setuptimemserror(error_data.uptime_ms);
     //qDebug()<<"Deive Id error "<<error_data.device_id;
    switch(error_data.device_id){
    case DEVICE_HEARTBEAT:
        //error_device = Device_HEARTBEAT;
        device_id = DEVICE_HEARTBEAT;
        error_device = "HEARTBEAT";
        break;
    case DEVICE_GPS:
        device_id=DEVICE_GPS;
        error_device = "GPS";
        break;
    case DEVICE_AHRS:
        device_id = DEVICE_AHRS;
        error_device = "AHRS";
        break;
    case DEVICE_IMU:
        device_id=DEVICE_IMU;
        error_device = "IMU";
        break;
    case DEVICE_EXTERNAL_ATMOSPHERE:
        device_id = DEVICE_EXTERNAL_ATMOSPHERE;
        error_device = "External Atmosphere";
        break;
    case DEVICE_SONAR:
        device_id = DEVICE_SONAR;
        error_device = "Sonar";
        break;
    case DEVICE_JOYSTICK:
        device_id = DEVICE_JOYSTICK;
        error_device = "Joystick";
        break;
    case DEVICE_POWER_HEALTH:
        device_id = DEVICE_POWER_HEALTH;
        error_device = "Power";
        break;
    case DEVICE_COMMAND_ACK:
        device_id = DEVICE_COMMAND_ACK;
        error_device = "Acknowledgement";
        break;
    case DEVICE_MOTOR:
        device_id = DEVICE_MOTOR;
        error_device = "Thrusters";
        break;

    }
    switch(error_data.error_type)
    {
    case DEVICE_ERROR_INIT:
        error_type=DEVICE_ERROR_INIT;
        err_type = "Initialization Error";
        break;
    case DEVICE_ERROR_HEARTBEAT:
        error_type=DEVICE_ERROR_HEARTBEAT;
        err_type = "Heartbeat Error";
        break;
    case DEVICE_ERROR_DATA_TRANSFER_FAILED:
        error_type=DEVICE_ERROR_DATA_TRANSFER_FAILED;
        err_type = "Data Transfer Failed";
        break;
    case DEVICE_ERROR_CRC_MISMATCH:
        error_type=DEVICE_ERROR_CRC_MISMATCH;
        err_type = "CRC Mismatch";
        break;
    case DEVICE_ERROR_TIMEOUT:
        error_type=DEVICE_ERROR_TIMEOUT;
        err_type = "Timeout Error";
        break;
    case DEVICE_ERROR_OUT_OF_RANGE:
        error_type=DEVICE_ERROR_OUT_OF_RANGE;
        err_type = "Value Out of Range";
        break;
    case DEVICE_ERROR_NOT_RESPONDING:
        error_type=DEVICE_ERROR_NOT_RESPONDING;
        err_type = "Not Responding";
        break;
    case DEVICE_ERROR_COMMUNICATION_LOST:
        error_type=DEVICE_ERROR_COMMUNICATION_LOST;
        err_type = "Communication Lost";
        break;
    case DEVICE_ERROR_CALIBRATION_FAILED:
        error_type=DEVICE_ERROR_CALIBRATION_FAILED;
        err_type = "Error Calibration Failed";
        break;
    case DEVICE_ERROR_UNKNOWN:
        error_type=DEVICE_ERROR_UNKNOWN;
        err_type = "Unknown Error";
       // emit
        break;
    }
    device_id=error_data.device_id;
    error_type=error_data.error_type;
    errorArray.clear();
    //errorArray.append(device_id);
    //qDebug()<<"error device id "<<device_id;
    //errorArray.append(error_type);
    //qDebug()<<"error_type"<<error_type;
    errorArray.append(error_device);
    errorArray.append(err_type);

    emit errordetected();
    // switch(error_data.severity)
    // {
    //     case

}

//void Link::
void Link::power_health_parsing()
{

    setBatteryVoltage(Power_health_data.battery_voltage);
}
void Link::motor_parsing()
{
    setMotorVelocity4(motor_data.velocity[3]);
}
void Link::sonar_parsing()
{
    setSonarRange(sonar_data.range_m);
    setSonarConfidence(sonar_data.confidence);
}
void Link::command_parsing()
{
    setCommand(acknowledgement_data.command);
    setResult(acknowledgement_data.result);
    setSource( acknowledgement_data.source);
    setFlagsAck(acknowledgement_data.flags);
    setMessage(acknowledgement_data.message);
    setUptimeMsAck(acknowledgement_data.uptime_ms);
}
float Link::yaw_value() const
{
    return yaw_deg;
}
float Link:: pitch_value() const
{
    return pitch_deg;
}
float Link::roll_value() const
{
    return roll_deg;
}
quint32 Link::uptime_ms_ahrs_value() const
{
    return uptime_ms_ahrs;
}
float Link::temperature() const
{
    //qDebug()<<temperature_C;
    return temperature_C;

}
float Link::depth() const
{
    return depth_m;
}
float Link::salinity () const
{
    return salinity_ppt;
}
uint8_t Link::deviceId() const
{
    return device_id;
}
uint8_t Link::errorType() const
{
    return error_type;
}
uint8_t Link:: error_severity() const
{
    return severity;
}
uint8_t Link::errorflags() const
{
    return flags;
}
quint64 Link::uptime_mserror() const
{
    return uptime_ms_error;
}
float Link:: motorVelocity4() const
{
    return velocity[3];
}
float Link::batteryVoltage() const
{
    return battery_voltage;
}
float Link::sonarRange() const
{
    return range_m;
}
float Link:: sonarConfidence() const
{
    return confidence;
}
uint8_t Link::commandValue() const
{
    return command;
}
uint8_t Link::resultValue() const
{
    return result;
}
uint8_t Link:: sourceValue() const
{
    return source;
}
uint8_t Link:: flagsAck() const
{
    return flags_ack;
}
QString Link:: messageValue() const
{
    return message;
}
quint64 Link:: uptimeMsAck() const
{
    return uptime_ms_ack;
}

void Link::save_short()
{
    if (!File.isOpen()) {
        writing_done=true;
        return;
    }
    QVector<QString> localCopy;
    {
        QMutexLocker locker(&mutex1);  // Locks until function exit
        localCopy = std::move(twoDArray);  // Transfers ownership
        twoDArray.clear();
        array_size=0;
    }


    QtConcurrent::run([this, localCopy]() {
        QMutexLocker streamLocker(&streamMutex);
        //QMutexLocker file_lock(&file_mutex);

        if (!File.isOpen()) return;

        for (const auto& item : localCopy) {
            stream << item;
            stream2 << item;
        }
        stream.flush();
        stream2.flush();

        writing_done=true;              // Consider making this atomic
    });
}

void Link::set_mode(int mode)
{
    switch(mode)
    {
    case 0:
        sendMode(MODE_STABILIZE);
        break;
    case 1:
        sendMode(MODE_DEPTH_HOLD);
        break;
    case 2:
        sendMode(MODE_HOLD);
        break;
    case 3:
        sendMode(MODE_MANUAL);
        break;
    default:
        sendMode(MODE_HOLD);
        break;
    }
}

void Link::gain_update(int value)
{
    gain_plus = value;
    //qDebug()<<gain_plus;
    switch (gain_plus) {
    case 4:
        emit gain_status(25);
        break;
    case 3:
        emit gain_status(50);
        break;
    case 2:
        emit gain_status(75);
        break;
    case 1:
        emit gain_status(100);
        break;
    default:
        emit gain_status(25);
        break;
    }
}

void Link::joystick_controller_updated(QString name,bool status)
{
    //qDebug()<<name;
    joystick_changed(status);
}

void Link::arm_disarm(int mode, bool value)
{
    //qDebug()<<mode;
    switch(mode)
    {
    case 0:
        //qDebug()<<"arm";
        arm_status = ARMED;
        send_arm(arm_status);
        break;
    case 1:
        arm_status = DISARMED;
        send_arm(arm_status);
        //qDebug()<<"Disarm";
        break;
    default:
        break;
    }
}

void Link::send_arm(ARM_State state)
{
    COMMAND_Packet cmd{};
    cmd.id=CMD_ARM_DISARM;
    cmd.target=DEVICE_COMMAND;
    cmd.flags=0;
    cmd.param_count=1;
    cmd.param[0]=state;
    cmd.uptime_ms   =static_cast<uint32_t>(QDateTime::currentDateTime().toMSecsSinceEpoch());
    //qDebug()<<"Arm Send:"<<state;

    VEDTP_Main COMMAND_send_Packet;


    uint8_t encode_status = encode_COMMAND(&COMMAND_send_Packet,&cmd);   // encode heartbeat function from vedtp protocol library
    //qDebug()<<encode_status;
    if(encode_status)
    {
        //mavlink_send_buffer(COMMAND_send_Packet);
        emit _invokeWriteBytes(
            QByteArray(reinterpret_cast<char*>(&COMMAND_send_Packet),
                       sizeof(VEDTP_Main))
            );
    }

}

int Link::mapvalue(double value, int InputMin, int InputMax, int outputMin,int outputMax)
{
    //qDebug()<<"Mapped Value";
    //delta_gain = (1900-1500)/gain_plus;
    delta_gain = 400/gain_plus;
    outputMax = 1500+ delta_gain;
    outputMin = 1500- delta_gain;
    //qDebug()<<gain_plus<<delta_gain<<outputMax<<outputMin;
    double mappedValue =
        (value - InputMin) * (outputMax - outputMin)
            / static_cast<double>(InputMax - InputMin)
        + outputMin;
    //qDebug()<<mappedValue;
    //Clamp the mapped value
    if (mappedValue < outputMin)
        return outputMin;
    else if (mappedValue > outputMax)
        return outputMax;
    else
        return mappedValue;

}
void Link::setYaw_value(float value)
{
    if (qFuzzyCompare(yaw_deg, value))
        return;

    yaw_deg = value;
    emit yawValueChanged(yaw_deg);
}
void Link:: setPitch_value(float value)
{
    if(qFuzzyCompare(pitch_deg, value))
        return;
    pitch_deg = value;
    emit pitchValueChanged(pitch_deg);
}
void Link::setRoll_value(float value)
{
    if(qFuzzyCompare(roll_deg, value))
        return;
    roll_deg = value;
    emit rollValueChanged(roll_deg);
}
void Link::setuptime_ms_ahrs_value(quint64 value)
{
    if(uptime_ms_ahrs==value)
        return;
    uptime_ms_ahrs = value;
    emit uptimeChanged(uptime_ms_ahrs);
}
void Link::settemperature(float value)
{
    if(qFuzzyCompare(temperature_C, value))
       return;
    temperature_C = value;
   // qDebug()<<"UI"<<temperature_C;
    emit temperatureChanged(temperature_C);
}
void Link::setdepth(float value)
{
    if(qFuzzyCompare(1.0+depth_m,1.0+ value))
       return;
    depth_m=value;
    emit depthChanged(depth_m);
}
void Link::setsalinity(float value)
{
    if(qFuzzyCompare(1.0+salinity_ppt,1.0+value))
        return;
    salinity_ppt = value;
    emit salinityChanged(salinity_ppt);
}
void Link:: setDeviceId(uint8_t value)
{
    if(device_id == value)
        return;
    device_id = value;
    emit deviceIdChanged(device_id);
}
void Link:: setErrorType(uint8_t value)
{
    if(error_type == value)
        return;
    error_type = value;
    emit errorTypeChanged(error_type);
}
void Link:: setFlags(uint8_t value)
{
    if(flags==value)
        return;
    flags = value;
    emit flagsChanged(flags);
}
void Link:: setseverity(uint8_t value)
{
    if(severity==value)
        return;
    severity=value;
    emit severityChanged(value);
}
void Link:: setuptimemserror(quint64 value)
{
    if(uptime_ms_error == value)
        return;
    uptime_ms_error = value;
    emit uptimemserrorChanged(uptime_ms_error);
}
void Link::setMotorVelocity4(float value)
{
    if(qFuzzyCompare(1.0+velocity[3],1.0+value))
        return;
    velocity[3]=value;
    emit motorVelocity4Changed(velocity[3]);
}
void Link::setBatteryVoltage(float value)
{
    if(qFuzzyCompare(1.0+battery_voltage,1.0+value))
        return;
    battery_voltage = value;
    emit batteryVoltageChanged(battery_voltage);
}
void Link::setSonarRange(float value)
{
    if(qFuzzyCompare(1.0+range_m,1.0+value))
        return;
    range_m = value;
    emit sonarRangeChanged(range_m);
}
void Link::setSonarConfidence(float value)
{
    if(qFuzzyCompare(1.0+confidence,1.0+value))
        return;
    confidence = value;
    emit sonarConfidenceChanged(confidence);
}
void Link::setCommand(uint8_t value)
{
    if(command==value)
        return;
    command = value;
    emit commandChanged(command);
}
void Link::setResult(uint8_t value)
{
    if(result==value)
        return;
    result = value;
    emit resultChanged(result);
}
void Link::setSource(uint8_t value)
{
    if(source==value)
        return;
    source = value;
    emit sourceChanged(source);
}
void Link::setFlagsAck(uint8_t value)
{
    if(flags_ack == value)
        return;
    flags_ack = value;
    emit flagsAckChanged(flags_ack);
}
void Link:: setMessage(const QString & value)
{
    if(message == value)
        return;
    message = value;
    emit messageChanged(message);
}
void Link::setUptimeMsAck(quint64 value)
{
    if(uptime_ms_ack == value)
        return;
    uptime_ms_ack = value;
    emit uptimeMsAckChanged(uptime_ms_ack);
}
void Link::generateRandomData()
{
    float randomYaw =
        QRandomGenerator::global()->generateDouble() * 360.0f - 180.0f;
    setYaw_value(randomYaw);
}
void Link::create_directory()
{

    //qDebug()<<"Directory created";

    QString targetDirectory = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
    QString logFileDir = targetDirectory + "/orcascreenshots";
    QDir().mkpath(logFileDir);
    /*QString targetDirectory2 = QStandardPaths::writableLocation(QStandardPaths::MoviesLocation);
    QString logFileDir2 = targetDirectory2 + "/orca videos";
    QDir().mkpath(logFileDir2);*/
    QString targetDirectory3 = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    log_path = targetDirectory3 + "/ORCA Logfiles";
    QDir().mkpath(log_path);
    /*QString targetDirectory4 = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    QString error_path = targetDirectory4 + "/ORCA ErrorFiles";
    QDir().mkpath(error_path);*/
    /*QString targetDirectory5 = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    QString pdf_path = targetDirectory5 + "/ORCA PdfFiles";
    QDir logDir(pdf_path);
    QDir().mkpath(pdf_path);*/

    // Get the target directory path for saving files.
    targetDirectory = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    // Modify the target directory to add a subdirectory for your files.
    logFileDir5 = targetDirectory + "/ORCA LogFiles";
    /*err_path2   = targetDirectory + "/ORCA CommandFiles";
    err_path3   = targetDirectory + "/ORCA ErrorFiles";
    bb_path     = targetDirectory + "/ORCA BB";*/

    // Create the directory if it doesn't exist.
    QDir().mkpath(logFileDir5);
   /* QDir().mkpath(err_path2);
    QDir().mkpath(err_path3);
    QDir().mkpath(bb_path);*/
}
void Link::sendJoystickData()
{
    /*qDebug() << "Joystick send called:"
             << QDateTime::currentMSecsSinceEpoch();*/
    startTime = timer2.elapsed();
    latency = startTime - elapse_time;
    //qDebug()<<latency<<prev_latency;
    JOYSTICK_Packet joy = {};
    joy.channels[0] = y_axis_left;
    joy.channels[1] = x_axis_left;
    joy.channels[2] = x_axis_right;
    joy.channels[3] = y_axis_right;
    joy.channels[4] = light_intensity;
    joy.channels[5] = light_intensity;
    joy.channels[6] = light_intensity;
    joy.channels[7] = light_intensity;
    /*qDebug()<<"channels 0: "<<joy.channels[0];
    qDebug()<<"channel 1: "<<joy.channels[1];
    qDebug()<<"chanel 2: "<<joy.channels[2];
    qDebug()<<"channel 3: "<<joy.channels[3];
    qDebug()<<"channel 4: "<<joy.channels[4];*/
    VEDTP_Main JOYSTICK_send_Packet{};
    joy.flags=0;
    joy.uptime_ms = static_cast<uint32_t>(QDateTime::currentDateTime().toMSecsSinceEpoch());

    uint8_t encode_status = encode_JOYSTICK(&JOYSTICK_send_Packet,&joy);   // encode heartbeat function from vedtp protocol library
    if(encode_status)
    {
    //mavlink_send_buffer(JOYSTICK_send_Packet);
        emit _invokeWriteBytes(
            QByteArray(reinterpret_cast<char*>(&JOYSTICK_send_Packet),
                       sizeof(VEDTP_Main))
            );
    }
    elapse_time = startTime;
    prev_latency = latency;

}
void Link::set_joystick_values(double y_axis_, double x_axis_)
{
    QMutexLocker locker(&mutex10);
    y_axis_left = mapvalue(y_axis_,InputMin, InputMax, outputMin, outputMax);
    x_axis_left = mapvalue(x_axis_,InputMin, InputMax, outputMin, outputMax);
    //qDebug()<<"set_joystick_values";
    //x_axis_left = static_cast<uint16_t>(x_axis_);
    //x_axis_right = static_cast<uint16_t>(x_axis1_);
    //y_axis_right = static_cast<uint16_t>(y_axis1_);
    //qDebug()<<"y_axis_left"<<y_axis_left;
    //qDebug()<<"x_axis_left"<<x_axis_left;
    //qDebug()<<x_axis_right;
    //qDebug()<<y_axis_right;
}
void Link::set_joystick_values2(double y_axis_, double x_axis_)
{
    QMutexLocker locker(&mutex11);
    y_axis_left = mapvalue(y_axis_,InputMin, InputMax, outputMin, outputMax);
    x_axis_left = mapvalue(x_axis_, InputMin, InputMax, outputMin, outputMax);
    /*qDebug()<<"set_joystick_values2";
   qDebug()<<"y_axis_left"<<y_axis_left;
    qDebug()<<"x_axis_left"<<x_axis_left;*/
}
void Link::set_joystick_values3(double  y_axis_, double x_axis_)
{
    QMutexLocker locker(&mutex12);
    y_axis_right = mapvalue(y_axis_, InputMin, InputMax, outputMin, outputMax);
    x_axis_right = mapvalue(x_axis_, InputMin, InputMax, outputMin, outputMax);
    /*qDebug()<<"set_joystick_values3";
    qDebug()<<"y_axis_right"<<y_axis_right;
    qDebug()<<"x_axis_right"<<x_axis_right;*/
}
void Link::set_joystick_values4(double y_axis_, double x_axis_)
{
    QMutexLocker locker(&mutex13);
    y_axis_right = mapvalue(y_axis_,InputMin, InputMax, outputMin, outputMax);
    x_axis_right = mapvalue(x_axis_,InputMin, InputMax, outputMin, outputMax);
    /*qDebug()<<"set_joystick_values4";
    qDebug()<<"y_axis_right"<<y_axis_right;
    qDebug()<<"x_axis_right"<<x_axis_right;*/
}
void Link::joystick_activated(int mode, bool value)
{
    if (mode == MODE_DEPTH_HOLD && value) {
        is_depth_hold = !is_depth_hold;
        COMMAND_Packet cmd = {};
        //vedtp_send = encode_COMMAND(&cmd);
        //qDebug() << "Depth Hold:" << is_depth_hold;
    }
}
void Link::sendMode(System_Mode mode)
{
    COMMAND_Packet cmd{};
    cmd.id          = CMD_MODE;
    cmd.target      = DEVICE_COMMAND;
    cmd.flags       = 0;
    cmd.param_count = 1;
    cmd.param[0]    = static_cast<int16_t>(mode);
    cmd.char1[12] = static_cast<uint8_t>(mode);
    cmd.char2[12] = static_cast<uint8_t>(mode);

    cmd.uptime_ms   = QDateTime::currentMSecsSinceEpoch();
    VEDTP_Main command_send_packet{};
    if (!encode_COMMAND(&command_send_packet, &cmd)) {
        qDebug() << "[MODE] Encode failed";
        return;
    }
    else
    {
        //mavlink_send_buffer(command_send_packet);
        emit _invokeWriteBytes(
            QByteArray(reinterpret_cast<char*>(&command_send_packet),
                       sizeof(VEDTP_Main))
            );
    }

    //qDebug() << "[MODE] Sent mode:" << mode;
}
void Link::setManualMode()
{
    sendMode(MODE_MANUAL);
}
void Link::setHoldMode()
{
    sendMode(MODE_HOLD);
}
void Link::setAutoMode()
{
    sendMode(MODE_AUTO);
}
void Link::setRTSMode()
{
    sendMode(MODE_RTL);
}
void Link::sendModeCommand(unsigned char mode)
{
    sendMode(static_cast<System_Mode>(mode));
}

QVariantList Link::errorcatched()
{
    return errorArray;
}
void Link::login(QString username, QString password)
{
    COMMAND_Packet cmd{};
    cmd.id=CMD_LOGIN;
    cmd.target=DEVICE_COMMAND;
    cmd.flags=0;
    cmd.param_count=2;
    username ="VOTPL";
    password ="VOTPL";
    strcpy(cmd.char1, username.toUtf8().constData());
    strcpy(cmd.char2, password.toUtf8().constData());
    //cmd.char1[12] ="votpl";
    cmd.uptime_ms   = QDateTime::currentMSecsSinceEpoch();
    VEDTP_Main heartbeat_send_packet;                                                   // for holding the main vedtp packet

    if (!encode_COMMAND(&vedtp, &cmd)) {
        return;
    }
    emit _invokeWriteBytes(
        QByteArray(reinterpret_cast<char*>(&vedtp),
                   sizeof(VEDTP_Main))
        );//cmd.param[0]=static_cast<>
}
