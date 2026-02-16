#ifndef JOYSTICKCONTROLLER_H
#define JOYSTICKCONTROLLER_H

#include <QObject>
#include <QTimer>
#include <SDL3/SDL.h>

class JoystickController : public QObject
{
    Q_OBJECT
public:
    explicit JoystickController(QObject *parent = nullptr);
    ~JoystickController();

    bool initialize();
    void shutdown();
signals:
    void leftaxisX(double value1, double value2);
    void leftaxisY(double value1, double value2);
    void rightaxisX(double value1, double value2);
    void rightaxisY(double value1, double value2);
    void arm_status(int,bool);
    void mode(int);
    void set_gain(int);
    void joystick_detected(QString,bool);
private slots:
    void pollEvents();
    void testjoystickparameter(double value1, double value2);
private:
    uint16_t mapAxisToPWM(float axisValue, int min = 1100, int max = 1900);
    float leftX  = 0.0f;
    float leftY  = 0.0f;
    float rightX = 0.0f;
    float rightY = 0.0f;
    //int SDL_NumJoysticks();
    QTimer timer;
    SDL_Joystick *myJoystick;
    bool arm_status_value=false;
    bool disarm_status_value=false;
    QString name="Unknown";
    int gain_add=4;
};

#endif // JOYSTICKCONTROLLER_H
