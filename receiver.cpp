#include "receiver.h"
#include<qDebug>
#include <QUdpSocket>
#include <qdatetime.h>
static QUdpSocket *send_socket;
//QUdpSocket* nmea_socket = nullptr;  // actual definition

static QHostAddress receiverAddress=QHostAddress("192.168.56.2");
static quint16 receiverPort = 10050, CPT_PORT = 10055;
Receiver::Receiver() {
    socket=new QUdpSocket(this);
    if (!socket->bind(QHostAddress::AnyIPv4, 10050)) {
        qDebug() << "Failed to bind UDP socket to" << Address << ":" << port;
    }
    /*QTimer *timer = nullptr;
    timer = new QTimer();
    connect(timer,& QTimer::timeout, this, &Receiver::receivedatagram);
    timer->start(1000);*/
    connect(socket,& QUdpSocket::readyRead, this, &Receiver::receivedatagram);


}

void Receiver::senddatagram(QByteArray buffer)
{
    //qDebug()<<buffer.toHex();
    //socket->writeDatagram(data,Address,port);
    // Implicit thread-safety via Qt's event loop
    /*qDebug() << "UDP actually sent:"
             << QDateTime::currentMSecsSinceEpoch();*/
   QMetaObject::invokeMethod(this, [this, buffer]() {
       if(!receiverAddress.isNull())
       {
           //qDebug()<<receiverAddress;
           //qDebug()<<receiverPort;
           //
           int written=socket->writeDatagram(buffer,receiverAddress, receiverPort);
           //qDebug()<<written;
           //qDebug()<<socket->readBufferSize();

       }
       //else
       ;
   }, Qt::DirectConnection);
    //qDebug()<<"message"<<message;
}

void Receiver::receivedatagram()
{
    //qDebug()<<"data";
    /*if(!socket)
        return;*/
    QByteArray Buffer;
    if(socket->state()==QUdpSocket::BoundState)
    {
        if(socket->hasPendingDatagrams())
        {
            datagram.clear();
            QHostAddress senderAddress;
            quint16 senderPort;
            datagram.resize(socket->pendingDatagramSize());
            socket->readDatagram(datagram.data(), datagram.size(),& senderAddress, &senderPort);
            Buffer.append(datagram);
            emit datareceived(Buffer);
            Buffer.clear();
        }
    }
    //socket->receiveDatagram(data)
}
