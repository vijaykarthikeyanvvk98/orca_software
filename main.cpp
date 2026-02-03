#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include "opencvimageprovider.h"
#include "joystickcontroller.h"
#include "videostreamer.h"
#include "link.h"
#include "receiver.h"
#include <QQmlContext>
#include <QFile>
int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;

    qRegisterMetaType<cv::Mat>("cv::Mat");

    VideoStreamer videoStreamer;
    //JoystickController joystickcontroller;
   // joystickcontroller.initialize();
    OpencvImageProvider *liveImageProvider(new OpencvImageProvider);
    Link link;
    link.create_directory();
    Receiver receiver;
    engine.rootContext()->setContextProperty("VideoStreamer",&videoStreamer);
    engine.rootContext()->setContextProperty("liveImageProvider",liveImageProvider);
    engine.rootContext()->setContextProperty("link", &link);
    engine.rootContext()->setContextProperty("receiver", &receiver);
   // engine.rootContext()->setContextProperty("joystickcontroller",&joystickcontroller);
    engine.addImageProvider("live",liveImageProvider);

    engine.load(QUrl(QStringLiteral("qrc:/resources/main.qml")));

    QObject::connect(&videoStreamer,&VideoStreamer::newImage,liveImageProvider,&OpencvImageProvider::updateImage);

    //QFile f("qrc:/resources/main.qml");
    //qDebug() << "Exists?" << f.exists();  // Should print true

    return app.exec();
}
