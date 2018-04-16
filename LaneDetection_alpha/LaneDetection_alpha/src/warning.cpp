#include "include/def.h"
#include "include/warning.h"


ULONG JudgeWarning(ROAD& road, vector<ROAD>& roads, INT& directDeparture)
{
    ULONG ret = HCOM_OK;
    INT warningTrh = 20;
    INT sople = 0;


    if (0 == road.middleLane.length)
    {
        return ret;
    }

    sople = roads.back().middleLane.s - road.middleLane.s;
    
    if (abs(sople) > warningTrh && sople > 0 && road.middleLane.s < 0)
    {
        directDeparture = DIRECT_RIGHT;
    }
    else if (abs(sople) > warningTrh && sople < 0 && road.middleLane.s < 0)
    {
        directDeparture = DIRECT_LEFT;
    }
    else
    {
        directDeparture = DIRECT_NONE;
    }



    return ret;
}