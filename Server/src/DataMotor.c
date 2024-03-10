#include <define.h>
int32_t position_speed[][5] =
{
    {Absolute,NoChange,NoChange,NoChange,NoChange}, // DO NOT DELETE THIS LINE
    {LOCK_MOTOR,Disable,Disable,Disable,Disable}, //DO NOT DELETE THIS LINE
    {TOP_SENSOR_STAT,Enable,Enable,Enable,Enable}, //DO NOT DELETE THIS LINE
    //**************Change the Code Bellow***************
    {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},
    {Absolute,1050000,1200000,0,11250000},
    // {Absolute,1050000,1200000,12000,18250000},// above sensor
    // {Reletive,NoChange,NoChange,700000,NoChange},// first touch motor 3
    // {Reletive,NoChange,NoChange,-20000,NoChange},

    // {Speed,NoChange,NoChange,7000,NoChange},
    // {Speed,NoChange,NoChange,7000,NoChange},

    // {Reletive,NoChange,NoChange,700000,NoChange},// exact touch motor 3

    // {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},

    // {Reletive,NoChange,NoChange,-100000,NoChange},
    // {Reletive,NoChange,-300000,NoChange,NoChange},
    // {Reletive,NoChange,NoChange,105000,NoChange},
    // {Reletive,NoChange,500000,NoChange,NoChange},// first touch motor 2
    // {Reletive,NoChange,NoChange,-100000,NoChange},
    // {Reletive,NoChange,-20000,NoChange,NoChange},

    // {Speed,NoChange,7000,NoChange,NoChange},
    // {Speed,NoChange,7000,NoChange,NoChange},

    // {Reletive,NoChange,NoChange,100000,NoChange},
    // {Reletive,NoChange,600000,NoChange,NoChange},// exact touch motor 2
    // {Reletive,NoChange,NoChange,-500000,NoChange},
    // {Reletive,NoChange,NoChange,NoChange,-7000000},

    // {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},
    
    // {Reletive,NoChange,3500000,NoChange,NoChange},
    // {Reletive,NoChange,239000,NoChange,NoChange},
    // {Reletive,NoChange,NoChange,672500,NoChange},// p1 tangent

    // {Speed,NoChange,NoChange,10000,NoChange},
    // {Speed,NoChange,NoChange,10000,NoChange},

    // {Reletive,NoChange,NoChange,170000,NoChange},// p1 grind
    // {Reletive,NoChange,-1327000,NoChange,NoChange},// p2 m2 back
    // {Reletive,140000,NoChange,NoChange,NoChange},// p2 m1 adjust

    // {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},
    // {Reletive,NoChange,NoChange,10000,NoChange},
    // {Reletive,NoChange,NoChange,762000,NoChange},// p2 tangent
   
    // {Speed,NoChange,NoChange,10000,NoChange},
    // {Speed,NoChange,NoChange,10000,NoChange},

    // {Reletive,NoChange,NoChange,10000,NoChange},// p2-1 grind

    // {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},

    // {Reletive,NoChange,NoChange,-208500,NoChange},// tabdil m3 
    // {Reletive,-350000,NoChange,NoChange,NoChange},// tabdil m1
    // {Reletive,NoChange,NoChange,137500,NoChange},// tangent p2-2

    // {Speed,NoChange,NoChange,10000,NoChange},
    // {Speed,NoChange,NoChange,10000,NoChange},

    // {Reletive,NoChange,NoChange,50000,NoChange},// grind p2-2
    
    // {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},
    {Speed,DefSpeed,DefSpeed,DefSpeed,DefSpeed},
    {First_Independent_absolute_Set,NoChange,NoChange,NoChange,8250000},
    {Last_Independent_absolute_Set,NoChange,NoChange,NoChange,6250000},
    {Start_IndependentMove,NoChange,NoChange,NoChange,true},
    {Motor_delay,13000000,NoChange,NoChange,NoChange},
    {STOP_IndependentMove,true,true,true,true},

    // {Absolute,NoChange,45000,45000,NoChange},// above sensor
    // {Absolute,30000,NoChange,NoChange,30000},// above sensor
    //DO NOT DELETE BELLOW LINE
    {LOCK_MOTOR,Disable,Disable,Disable,Disable}, //DO NOT DELETE THIS LINE
    {TOP_SENSOR_STAT,Enable,Enable,Enable,Enable}, //DO NOT DELETE THIS LINE
    {END_OF_Command,NoChange,NoChange,NoChange,NoChange},//DO NOT DELETE THIS LINE
    {END_OF_Command,NoChange,NoChange,NoChange,NoChange},//DO NOT DELETE THIS LINE
};