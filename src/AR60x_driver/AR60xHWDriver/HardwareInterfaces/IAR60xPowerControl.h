#pragma once

#include "../DataTypes/PowerData.h"

class IAR60xPowerControl
{
public:
    virtual void PowerSetSettings(PowerData settings) = 0;
    virtual void SupplySetState(PowerData::PowerSupplies supply, bool onOffState) = 0;
};

