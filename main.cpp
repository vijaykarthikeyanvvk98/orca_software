#include <QApplication>
#include <QQmlApplicationEngine>
#include "opencvimageprovider.h"
#include "joystickcontroller.h"
#include "videostreamer.h"
#include "link.h"
#include "receiver.h"
#include <QQmlContext>
#include <QWindow>
#include <QFile>
#include "qicon.h"
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

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
    QObject *rootObject = engine.rootObjects().first();
    QWindow *window = qobject_cast<QWindow *>(rootObject);
    const QIcon icon = QIcon(":/resources/images/vikra_2.jpeg");
    if (window) {
        window->setIcon(icon);
        //window->setTitle("VIKRA ACV control station");
        window->showMaximized();
        window->setMinimumSize(QSize(670, 470));
    }
    return app.exec();
}
