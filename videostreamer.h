#ifndef VIDEOSTREAMER_H
#define VIDEOSTREAMER_H

#include <QObject>
#include <QTimer>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <QImage>
#include <QFile>
#include <iostream>
#include <QThread>
#include <QTextStream>
#include <QDateTime>
static cv::VideoCapture cap;


class VideoStreamer: public QObject
{
    Q_OBJECT
public:
    VideoStreamer();
    ~VideoStreamer();
    Q_INVOKABLE void pauseStreaming();
    Q_INVOKABLE void resumeStreaming();
public:
    void streamVideo();
    QThread* threadStreamer = new QThread();
    void catchFrame(cv::Mat emittedFrame);

public slots:
    void openVideoCamera(QString path);
    void closeCamera();
    void streamerThreadSlot();
    void changeCamera(QString path);
    void start_recording();
    void stop_recording();
    void subtitle_streaming();
    void set_ahrs(QVariantList);
private:
    cv::Mat frame;
    QTimer tUpdate;
    QFile subtitleFile;
    QTextStream out;
    QDateTime date;
    QString formattedTime;
    QByteArray formattedTimeMsg;
    int FPS_count = 0;
    double fps=30.00,fps2=30.00;
    cv::VideoWriter video;
    //QString FPS, YAW, PITCH, ROLL, TEMP, PRESSURE, DEPTH, BATTERY;
    cv::Point textPos;

//signals:
signals:
    void newImage(const QImage &image);
    void emitThreadImage(cv::Mat frameThread);
    void stop();
};

#endif // VIDEOSTREAMER_H
