#include <define.h>
int32_t position_speed[][5] =
{
    {Absolute,NoChange,NoChange,NoChange,NoChange}, // DO NOT DELETE THIS LINE
    {LOCK_MOTOR,Disable,Disable,Disable,Disable}, //DO NOT DELETE THIS LINE
    {TOP_SENSOR_STAT,Enable,Enable,Enable,Enable}, //DO NOT DELETE THIS LINE
    //**************Change the Code Bellow***************
    {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},
    {Absolute,5000,5000,5000,5000},
    {StallExecution,NoChange,NoChange,NoChange,NoChange},
    {Reletive,NoChange,NoChange,700000,12000},// first touch motor 3
    // {Reletive,NoChange,NoChange,-20000,NoChange},

    //DO NOT DELETE BELLOW LINE
    {LOCK_MOTOR,Disable,Disable,Disable,Disable}, //DO NOT DELETE THIS LINE
    {TOP_SENSOR_STAT,Enable,Enable,Enable,Enable}, //DO NOT DELETE THIS LINE
    {END_OF_Command,NoChange,NoChange,NoChange,NoChange},//DO NOT DELETE THIS LINE
    {END_OF_Command,NoChange,NoChange,NoChange,NoChange},//DO NOT DELETE THIS LINE
};