#ifndef CONTROL_MODEL
#define CONTROL_MODEL

struct ControlModel
{
    ControlModel()
    {
        Pitch = 0;
        Roll = 0;
        Yaw = 0;
        Throttle = 0;
    }

    float Pitch;
    float Roll;
    float Yaw;
    int Throttle;
};
#endif