#include <define.h>
int32_t position_speed[][5] =
{
    {Absolute,NoChange,NoChange,NoChange,NoChange}, // DO NOT DELETE THIS LINE
    {StopMotorMoving,Enable,Enable,Enable,Enable}, // Do NOT DELETE THIS LINE
    {LOCK_MOTOR,Disable,Disable,Disable,Disable}, //DO NOT DELETE THIS LINE
    {TOP_SENSOR_STAT,Enable,Enable,Enable,Enable}, //DO NOT DELETE THIS LINE
    //**************Change the Code Bellow***************
    {Speed,25000,25000,25000,25000},
    {Absolute,10000,5000,10000,10000},
    {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},
    {Reletive,NoChange,700000LL,700000LL,120000LL},
    {StallExecution,NoChange,NoChange,NoChange,10000LL}, 
    {Reletive,NoChange,NoChange,7000000LL,1200000LL},
    {Reletive,NoChange,NoChange,7000000LL,1200000LL},
    {Speed,35000,35000,35000,35000},
    {Reletive,7000000LL,7000000LL,7000000LL,1200000LL},

    //DO NOT DELETE BELLOW LINE
    {LOCK_MOTOR,Disable,Disable,Disable,Disable}, //DO NOT DELETE THIS LINE
    {TOP_SENSOR_STAT,Enable,Enable,Enable,Enable}, //DO NOT DELETE THIS LINE
    {END_OF_Command,NoChange,NoChange,NoChange,NoChange},//DO NOT DELETE THIS LINE
    {END_OF_Command,NoChange,NoChange,NoChange,NoChange},//DO NOT DELETE THIS LINE
};



int32_t home_pos[][5] =
{
    {Absolute,NoChange,NoChange,NoChange,NoChange}, // DO NOT DELETE THIS LINE
    {StopMotorMoving,Enable,Enable,Enable,Enable}, // Do NOT DELETE THIS LINE
    {LOCK_MOTOR,Disable,Disable,Disable,Disable}, //DO NOT DELETE THIS LINE
    {TOP_SENSOR_STAT,Enable,Enable,Enable,Enable}, //DO NOT DELETE THIS LINE
    //****************** Start Homing *******************
    {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},
    {GoHomeCommand,NoChange,NoChange,NoChange,Enable},
    {GoHomeCommand,Enable,Enable,Enable,NoChange},
    {Absolute,290000LL,290000LL,290000LL,290000LL},
    {Speed,500,150,500,500},
    {GoHomeCommand,Enable,Enable,Enable,Enable},
    {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},

    //DO NOT DELETE BELLOW LINE
    {LOCK_MOTOR,Disable,Disable,Disable,Disable}, //DO NOT DELETE THIS LINE
    {TOP_SENSOR_STAT,Enable,Enable,Enable,Enable}, //DO NOT DELETE THIS LINE
    {END_OF_Command,NoChange,NoChange,NoChange,NoChange},//DO NOT DELETE THIS LINE
    {END_OF_Command,NoChange,NoChange,NoChange,NoChange},//DO NOT DELETE THIS LINE
};