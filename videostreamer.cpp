#include "videostreamer.h"
#include <QDebug>
#include <QStandardPaths>
#include <QDateTime>
#include <QDir>
#include <opencv2/opencv.hpp>
//static bool recording_status = false,recording_status2 = false;
QTimer sub_timer;
QString subtitle_path = "";
static int sub_i=1;
static int sub_i2=2;
 bool recording_status = false;
int frame_width=0;
int frame_height=0;
std::string outputPathStdString="";
QString record_type="";
QString path ="";
VideoStreamer *worker ;
static qfloat16 yaw1 = 0.0;
static qfloat16 pitch1 = 0.0;
static qfloat16 roll1 = 0.0;
static qfloat16 yaw2 = 0;
static qfloat16 pitch2 = 0;
static qfloat16 roll2 = 0;
static QString FPS, YAW, PITCH, ROLL, TEMP, PRESSURE, DEPTH, BATTERY;

VideoStreamer::VideoStreamer()
{
    connect(&tUpdate,&QTimer::timeout,this,&VideoStreamer::streamVideo);
    connect(&sub_timer, &QTimer::timeout, this, &VideoStreamer::subtitle_streaming);
    YAW = "Heading:" + QString::number(yaw2, 'f', 1);
    PITCH = "Pitch:" + QString::number(pitch2, 'f', 1);
    ROLL = "Roll:" + QString::number(roll2, 'f', 1);

}
VideoStreamer::~VideoStreamer()
{
    frame=0;
    formattedTimeMsg.clear();
    if(threadStreamer && threadStreamer->isRunning())
    {
        threadStreamer->requestInterruption();  // Signal worker loop to stop
        threadStreamer->quit();
        threadStreamer->wait();
    }

    if(cap.isOpened())
        cap.release();
    worker=nullptr;

    recording_status=false;
    if (video.isOpened())
        video.release();

    if(subtitleFile.isOpen())
    {
        if(sub_timer.isActive())
        {
            sub_timer.stop();
        }
        out.flush();
        subtitleFile.close();
    }

    if(tUpdate.isActive())
    {
        tUpdate.stop();

    }

}
void VideoStreamer::streamVideo()
{

    if (!frame.empty()) {
        //qDebug()<<frame.cols<<frame.rows;
        //QImage img = QImage(frame.data, video_quality_x, video_quality_y, QImage::Format_RGB888).rgbSwapped();
        QImage img = QImage(frame.data, frame.cols, frame.rows, QImage::Format_RGB888).rgbSwapped();
        emit newImage(img);

        //qDebug()<<"Frame"<<recording_status;
    } else {
        qDebug() << "Frame empty";
    }
}
void VideoStreamer::catchFrame(cv::Mat emittedFrame)
{
    frame = emittedFrame;

    if (recording_status)
    {
        video.write(frame);
    }
}
void VideoStreamer::openVideoCamera(QString path)
{
    closeCamera();

    if(path.length() == 1)
        cap.open(path.toInt());
    else
        cap.open(path.toStdString(),cv::CAP_FFMPEG);

    if (!cap.isOpened()) {
        qDebug() << "error occured";
        return;
    }
    else
        ;
    worker = new VideoStreamer();
    worker->moveToThread(threadStreamer);
    QObject::connect(threadStreamer,SIGNAL(started()),worker,SLOT(streamerThreadSlot()));
    QObject::connect(worker,&VideoStreamer::emitThreadImage,this,&VideoStreamer::catchFrame);
    connect(threadStreamer, &QThread::finished, worker, &QObject::deleteLater);
    connect(threadStreamer, &QThread::finished, threadStreamer, &QObject::deleteLater);
    threadStreamer->start();
    if (cap.get(cv::CAP_PROP_FPS) <= 0)
    {
        tUpdate.start(1000 / 30);
        fps = 25;

    }
    else
    {
        tUpdate.start(1000 / cap.get(cv::CAP_PROP_FPS));
        fps =cap.get(cv::CAP_PROP_FPS);
        //qDebug()<<fps;
    }
    tUpdate.start(1000/fps);
}
void VideoStreamer::closeCamera()
{
    if (tUpdate.isActive())
        tUpdate.stop();
    if(cap.isOpened())
        cap.release();
    if (video.isOpened())
        video.release();
    //qDebug() << "Camera closed successfully";
}
void VideoStreamer::streamerThreadSlot()
{
    cv::Mat tempFrame;
    while (!QThread::currentThread()->isInterruptionRequested()) {
        if(threadStreamer->isInterruptionRequested())
        {
            cap.release();
            return;
        }
        cap>>tempFrame;
    if(tempFrame.data)
            emit emitThreadImage(tempFrame);

    }
}
void VideoStreamer::changeCamera(QString path)
{
    if(cap.isOpened())
    {
        cap.release();
    }

    if (path.length() == 1)
    {
        cap.open(path.toInt());
        //emit open_finished();
    }
    else
    {
        cap.open(path.toStdString());

    }

    if (!cap.isOpened()) {
        qDebug() << "error occured";
    }
    else
        ;
}
void VideoStreamer::start_recording()
{
    if(subtitleFile.isOpen())
    {
        sub_timer.stop();
        out.flush();
        subtitleFile.close();
        //sub_i=1;
        //sub_i2=2;
        sub_i=0;
        sub_i2=0;
    }
    recording_status = true;
    if(frame_width == 0)
    {
        frame_width= cap.get(cv::CAP_PROP_FRAME_WIDTH);
        frame_height= cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    }

    // Get the target directory path for saving files.
    QString targetDirectory = QStandardPaths::writableLocation(QStandardPaths::MoviesLocation);
    // Modify the target directory to add a subdirectory for your files.
    QString logFileDir = targetDirectory + "/Recording";

    // Create the directory if it doesn't exist.
    QDir().mkpath(logFileDir);

    if(path != "")
        logFileDir = path;
    QDateTime currentDateTime = QDateTime::currentDateTime();
    formattedTime = currentDateTime.toString("dd.MM.yyyy-hh.mm.ss");
    QString outputPath;
    // Construct the file paths relative to the target directory.
    if(record_type == "")
        outputPath = logFileDir + "/Recording-" + formattedTime + ".mp4";
    else
        outputPath = logFileDir + "/Recording-" + formattedTime + "." + record_type;

    QString outputPath2 = logFileDir + "/Recording-" + formattedTime + ".ass";
    subtitleFile.setFileName(outputPath2);
    outputPathStdString= outputPath.toStdString();
    if (subtitleFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        out.setDevice(&subtitleFile);
        // Write the ASS file header and styles
        qDebug() << "[Script Info]\n";
        qDebug()<< "Title: Example ASS File\n";
        qDebug() << "Original Script: OpenAI\n";
        qDebug() << "ScriptType: v4.00+\n";
        qDebug() << "Collisions: Normal\n";
        qDebug() << "PlayDepth: 0\n\n";
        qDebug() << "[V4+ Styles]\n";
        qDebug() << "Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, BackColour, Bold, Italic, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding\n";
        qDebug() << "Style: Default,Arial,20,&H00FFFFFF,&H00000000,&H00000000,-1,0,1,1.0,0.0,2,10,10,10,1\n\n";
        qDebug() << "[Events]\n";
        qDebug() << "Format: Marked, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text\n";
    }
    video.open(outputPathStdString,
              // cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
               cv::VideoWriter::fourcc('M', 'P', '4', 'V'),
               fps,
               cv::Size(frame_width,frame_height),
               true);
    //recording_status=true;
    if (!video.isOpened()) {
        qDebug() << "Error: Could not open the video writer.";
        recording_status = false;
    }
    else
    {
        recording_status = true;
    }
    sub_timer.start(1000);
}
void VideoStreamer::stop_recording()
{
    recording_status = false;
    //QThread::msleep(2000);
    if (video.isOpened()) {
        video.release();
        //export_video();
    }
    else
    {
    }
    //sub_i=1;
    //sub_i2=2;
    if(!video.isOpened())
    {
        if(subtitleFile.isOpen())
        {
            sub_timer.stop();
            out.flush();
            subtitleFile.close();
            sub_i=1;
            sub_i2=2;
        }
    }
}

void VideoStreamer::subtitle_streaming()
{
    if (subtitleFile.isOpen())
    {
        //qDebug()<<"angle";
        // Create subtitle text with sensor value
        //subtitleText = "Sensor Value: " + QString::number(sub_i);

        // Write subtitle to the .ass file for all four corners
        out << "Dialogue: 0,00:00:" + QString::number(sub_i) + ".00,00:00:" + QString::number(sub_i2) + ".00,Default,,0,0,0,,"
            << "{\\pos(50,20)}" // Top Left
            << "{\\fs" + QString::number(10) + "}"
            << YAW + "\n";

        /*if(WaterDepths != "")
             {
                 for(int i=0;i< dep_arr.length();i++)
                 {
                     assign_string = dep_arr[i];
                     if(i==0)
                     {
                 out << "Dialogue: 0,00:00:" + QString::number(sub_i) + ".00,00:00:" + QString::number(sub_i2) + ".00,Default,,0,0,0,,"
                     << "{\\pos(50,"+QString::number(20+0.05*frame_height,'f',0)+")}"
                     << "{\\fs" + QString::number(10) + "}"
                     << assign_string + "\n";
                     }
                     else
                     {
                         out << "Dialogue: 0,00:00:" + QString::number(sub_i) + ".00,00:00:" + QString::number(sub_i2) + ".00,Default,,0,0,0,,"
                             << "{\\pos(50,"+QString::number(20+0.05*frame_height+i*0.05*frame_height,'f',0)+")}"
                             << "{\\fs" + QString::number(10) + "}"
                             << assign_string + "\n";
                     }
                 }
             }*/

        out << "Dialogue: 0,00:00:" + QString::number(sub_i) + ".00,00:00:" + QString::number(sub_i2) + ".00,Default,,0,0,0,,"
            << "{\\pos("+QString::number(0.075*frame_width,'f',0)+",20)}" // Top Right
            << "{\\fs" + QString::number(10) + "}"
            << PITCH + "\n";

        /*out << "Dialogue: 0,00:00:" + QString::number(sub_i) + ".00,00:00:" + QString::number(sub_i2) + ".00,Default,,0,0,0,,"
            << "{\\pos(50,"+QString::number(0.9*frame_height,'f',0)+")}" // Bottom Left
            << "{\\fs" + QString::number(10) + "}"
            << ROLL + "\n";*/
        out << "Dialogue: 0,00:00:" + QString::number(sub_i) + ".00,00:00:" + QString::number(sub_i2) + ".00,Default,,0,0,0,,"
            << "{\\pos("+QString::number(0.125*frame_width,'f',0)+",20)}" // Top Right
            << "{\\fs" + QString::number(10) + "}"
            << ROLL + "\n";

        //out << "Picture: 1,0:00:13.00,0:00:16.00,,Football match 2015.09,17,17,3,,C:\\Users\\vijay\\Downloads\\sonar_arc_green.png\n";
        /*out << "Dialogue: 0,00:00:" + QString::number(sub_i) + ".00,00:00:" + QString::number(sub_i2) + ".00,Default,,0,0,0,,"
            << "{\\p1}m 10 10 l 50 10 l 50 50 l 10 50{\p0}\n"; */ // Example: Draws a square

        /*out << "Dialogue: 0,00:00:" + QString::number(sub_i) + ".00,00:00:" + QString::number(sub_i2) + ".00,Default,,0,0,0,,"
            << "{\\pos("+QString::number(0.7*frame_width,'f',0)+","+QString::number(0.9*frame_height,'f',0)+")}" // Bottom Right
            << "{\\fs" + QString::number(10) + "}"
            << PITCH + "\n";*/

        sub_i++;
        sub_i2++;
    }
}

void VideoStreamer::set_ahrs(QVariantList array)
{
    //qDebug()<<array[0];
    yaw2 = array[0].toFloat();
    pitch2 = array[1].toFloat();
    roll2 = array[2].toFloat();
    YAW = "Heading:" + QString::number(yaw2, 'f', 1);
    PITCH = "Pitch:" + QString::number(pitch2, 'f', 1);
    ROLL = "Roll:" + QString::number(roll2, 'f', 1);
    //qDebug()<<YAW<<PITCH<<ROLL;
    //press = array[1].toFloat();
    //depth = array[2].toFloat();
    //temp = array[3].toFloat();
    //battery_percentage = array[4].toInt();
}
void VideoStreamer::pauseStreaming()
{
    /*if (tUpdate.isActive()) {
        tUpdate.stop();
        qDebug() << "Video streaming paused";
    }*/
    recording_status = !recording_status;

}
void VideoStreamer::resumeStreaming()
{
    if (!tUpdate.isActive() && cap.isOpened()) {
        tUpdate.start(1000 / fps);
        qDebug() << "Video streaming resumed";
    }
}
