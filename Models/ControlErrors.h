#ifndef CONTROL_ERRORS
#define CONTROL_ERRORS

struct ControlErrors
{
    ControlErrors()
    {

        ControlRoll = 0;
        ControlPitch = 0;
        ControlYaw = 0;
    }

    float ControlRoll;
    float ControlPitch;
    float ControlYaw;
};

#endif