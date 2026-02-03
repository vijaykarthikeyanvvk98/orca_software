#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <QString>

class DataLogger
{
public:
    static void logIncoming(const QString &source, const QString &data);
    static void logOutgoing(const QString &destination, const QString &data);

private:
    static void writeLogs(const QString &direction,
                          const QString &endpoint,
                          const QString &data);
};

#endif
