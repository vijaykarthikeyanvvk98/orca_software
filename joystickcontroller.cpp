#include "joystickcontroller.h"
#include <QDebug>
#include <cmath>
JoystickController::JoystickController(QObject *parent)
    :QObject(parent),myJoystick(nullptr)
{
    connect(&timer, &QTimer::timeout,this, &JoystickController::pollEvents);
    connect(this,&JoystickController::leftaxisX,this,&JoystickController::testjoystickparameter);
}

JoystickController::~JoystickController()
{
    shutdown();
}
bool JoystickController::initialize()
{
    if (!SDL_Init(SDL_INIT_JOYSTICK))
    {
        qDebug() << "SDL could not initialize error:" << SDL_GetError();
        return false;
    }
    timer.start(16);
    return true;
}
void JoystickController::shutdown()
{
    timer.stop();
    if (myJoystick)
    {
        SDL_CloseJoystick(myJoystick);
        myJoystick = nullptr;
    }
    SDL_Quit();
}
uint16_t JoystickController::mapAxisToPWM(float axisValue, int min, int max)
{
    return static_cast<uint16_t>(
    std::round(min + ((axisValue + 1.0f) / 2.0f) * (max - min)));
}
void JoystickController::pollEvents()
{
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        if (event.type == SDL_EVENT_JOYSTICK_ADDED)
        {
            myJoystick = SDL_OpenJoystick(event.jdevice.which);
            if (myJoystick)
            {
                qDebug() << "Joystick Connected";
                qDebug() << "Name:" << SDL_GetJoystickName(myJoystick);
            }
        }
        else if (event.type == SDL_EVENT_JOYSTICK_BUTTON_DOWN)
        {
            qDebug() << "Button pressed:" << event.jbutton.button;
        }
        else if (event.type == SDL_EVENT_JOYSTICK_AXIS_MOTION)
        {
            const int AXIS_DEADZONE = 10;
            if (std::abs(event.jaxis.value) > AXIS_DEADZONE)
            {
                float norm =
                    (event.jaxis.value < 0)
                        ? event.jaxis.value / 32768.0f
                        : event.jaxis.value / 32767.0f;
                uint16_t pwmValue;
                /*if (event.jaxis.axis == 1) // vertical axis
                    pwmValue = mapAxisToPWM(-axisNormalized);
                else
                    pwmValue = mapAxisToPWM(axisNormalized);*/
                QString axisName;
                switch (event.jaxis.axis)
                {
                case 0:
                    leftX = norm;
                    axisName = "Left Stick X";
                    emit leftaxisX(leftY, leftX);
                  // qDebug()<<"Left Stick X: "<<leftX<<"Y:"<<leftY;
                    break;
                case 1:
                    leftY=-norm;
                    axisName = "Left Stick Y";
                    emit leftaxisY(leftY, leftX);
                   //qDebug()<<"Left Stick -> X"<<leftX<<"Y: "<<leftY;
                    break;
                case 3:
                    rightX=norm;
                    axisName = "Right Stick X";
                    emit rightaxisX(rightY, rightX);
                   // qDebug()<<"Right Stick-> X"<<rightX<<"Y: "<<rightY;
                    break;
                case 4:
                    rightY = -norm;
                    axisName = "Right Stick Y";
                    emit rightaxisY(rightY, rightX);
                  // qDebug()<<"Right Stick -> X: "<<rightX <<"Y: "<<rightY;
                    break;
                //case 4:

                default:
                    axisName = "Other Axis";
                    break;
                }
                      /* qDebug() << axisName
                         << "| Axis:" << event.jaxis.axis
                         << "| Raw:" << event.jaxis.value
                         << "| PWM:" << pwmValue;*/
            }
        }
        else if (event.type == SDL_EVENT_JOYSTICK_HAT_MOTION)
        {
            switch (event.jhat.value)
            {
            case SDL_HAT_UP: qDebug() << "D-pad Up"; break;
            case SDL_HAT_DOWN: qDebug() << "D-pad Down"; break;
            case SDL_HAT_LEFT: qDebug() << "D-pad Left"; break;
            case SDL_HAT_RIGHT: qDebug() << "D-pad Right"; break;
            case SDL_HAT_LEFTUP: qDebug() << "D-pad Left + Up"; break;
            case SDL_HAT_LEFTDOWN: qDebug() << "D-pad Left + Down"; break;
            case SDL_HAT_RIGHTUP: qDebug() << "D-pad Right + Up"; break;
            case SDL_HAT_RIGHTDOWN: qDebug() << "D-pad Right + Down"; break;
            default: break;
            }
        }
        else if (event.type == SDL_EVENT_JOYSTICK_REMOVED)
        {
            qDebug() << "Joystick Disconnected";
            SDL_CloseJoystick(myJoystick);
            myJoystick = nullptr;
        }
    }
}

void JoystickController::testjoystickparameter(double value1, double value2)
{
   // qDebug()<<value1<<value2;
}
