#include "receiver.h"
#include<qDebug>
#include <QUdpSocket>
Receiver::Receiver() {
    socket=new QUdpSocket(this);
    if (!socket->bind(QHostAddress(Address), port)) {
        qDebug() << "Failed to bind UDP socket to" << Address << ":" << port;
    }
    connect(socket,& QUdpSocket::readyRead, this, &Receiver::receivedatagram);


}

void Receiver::senddatagram(QByteArray data)
{
   // qDebug()<<data.toHex();
    //socket->writeDatagram(data,Address,port);
    int message = socket->writeDatagram(data,QHostAddress("192.168.1.12"),10050);
    //qDebug()<<"messag"<<message;
}

void Receiver::receivedatagram()
{
    if(!socket)
        return;
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
            //qDebug()<<datagram;
            emit datareceived(Buffer);
            Buffer.clear();
        }
    }
    //socket->receiveDatagram(data)
}
