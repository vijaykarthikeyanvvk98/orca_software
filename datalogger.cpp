#include "datalogger.h"
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QMutex>

static QMutex logMutex;

void DataLogger::logIncoming(const QString &source, const QString &data)
{
    writeLogs("IN", source, data);
}

void DataLogger::logOutgoing(const QString &destination, const QString &data)
{
    writeLogs("OUT", destination, data);
}

void DataLogger::writeLogs(const QString &direction,
                           const QString &endpoint,
                           const QString &data)
{
    QMutexLocker locker(&logMutex);

    QString time = QDateTime::currentDateTime()
                       .toString("yyyy-MM-dd HH:mm:ss.zzz");
     QFile txtFile("communication_log.txt");
    if (txtFile.open(QIODevice::Append | QIODevice::Text)) {
        QTextStream txt(&txtFile);
        txt << time
            << " [" << direction << "] "
            << endpoint << " : "
            << data
            << "\n";
    }
    QFile csvFile("communication_log.csv");
    bool newCsv = !csvFile.exists();

    if (csvFile.open(QIODevice::Append | QIODevice::Text)) {
        QTextStream csv(&csvFile);

        if (newCsv) {
            csv << "Time,Direction,Endpoint,Data\n";
        }

        QString safeData = data;
        safeData.replace("\"", "\"\"");  // CSV safety

        csv << time << ","
            << direction << ","
            << endpoint << ","
            << "\"" << safeData << "\"\n";
    }
}
