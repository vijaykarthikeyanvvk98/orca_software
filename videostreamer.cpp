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
VideoStreamer::VideoStreamer()
{
    connect(&tUpdate,&QTimer::timeout,this,&VideoStreamer::streamVideo);
}
VideoStreamer::~VideoStreamer()
{
    cap.release();
    tUpdate.stop();
    threadStreamer->requestInterruption();
}
void VideoStreamer::streamVideo()
{

    if(frame.data)
    {
        QImage img = QImage(frame.data,frame.cols,frame.rows,QImage::Format_RGB888).rgbSwapped();
        emit newImage(img);
    }
}
void VideoStreamer::catchFrame(cv::Mat emittedFrame)
{
    frame = emittedFrame;

    if (recording_status && video.isOpened())
    {
        if (frame.cols != frame_width || frame.rows != frame_height)
        {
            cv::resize(frame, frame, cv::Size(frame_width, frame_height));
        }

        video.write(frame);
    }
}
void VideoStreamer::openVideoCamera(QString path)
{
    closeCamera();
   //if(cap.isOpened())
      //  cap.release();
    if(path.length() == 1)
        cap.open(path.toInt());
    else
        cap.open(path.toStdString(),cv::CAP_FFMPEG);
    VideoStreamer* worker = new VideoStreamer();
    worker->moveToThread(threadStreamer);
    QObject::connect(threadStreamer,SIGNAL(started()),worker,SLOT(streamerThreadSlot()));
    QObject::connect(worker,&VideoStreamer::emitThreadImage,this,&VideoStreamer::catchFrame);
    threadStreamer->start();
    fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0 || fps > 120)
        fps = 30;
    tUpdate.start(1000/fps);
}
void VideoStreamer::closeCamera()
{
    if (tUpdate.isActive())
        tUpdate.stop();
    if(cap.isOpened())
        cap.release();
    qDebug() << "Camera closed successfully";
}
void VideoStreamer::streamerThreadSlot()
{
    cv::Mat tempFrame;
    while (1) {
        cap>>tempFrame;
    if(tempFrame.data)
            emit emitThreadImage(tempFrame);
    if(QThread::currentThread()->isInterruptionRequested())
        {
            cap.release();
            return;
        }
    }
}
void VideoStreamer::changeCamera()
{
    //qDebug()<<"camera changed";
    /*if(cap.isOpened())
    {
        cap.release();
        qDebug()<<"camera changed to rtsp";
    }*/
    openVideoCamera(
        "rtsp://admin:Vikra@123@192.168.56.50:554/video/live?channel=1&subtype=0"
        );
}
void VideoStreamer::start_recording()
{
    if(subtitleFile.isOpen())
    {
        sub_timer.stop();
        out.flush();
        subtitleFile.close();
        sub_i=1;
        sub_i2=2;
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
}
void VideoStreamer::pauseStreaming()
{
    if (tUpdate.isActive()) {
        tUpdate.stop();
        qDebug() << "Video streaming paused";
    }
}
void VideoStreamer::resumeStreaming()
{
    if (!tUpdate.isActive() && cap.isOpened()) {
        tUpdate.start(1000 / fps);
        qDebug() << "Video streaming resumed";
    }
}
