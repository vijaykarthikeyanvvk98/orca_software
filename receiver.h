#ifndef RECEIVER_H
#define RECEIVER_H
#include <QObject>
#include <QHostAddress>
#include <QUdpSocket>
#include <QTimer>
class Receiver:public QObject
{
    Q_OBJECT
public:
    Receiver();
public slots:
    void senddatagram(QByteArray data);
    void receivedatagram();
signals:
    void datareceived(QByteArray);

private:
    QByteArray data;
    QUdpSocket* socket;
    QTimer timer;
    QHostAddress senderaddress;
    quint16 senderport;
    QHostAddress Address=QHostAddress("192.168.1.17");
    quint16 port=10050;
    QByteArray datagram;
};
#endif
