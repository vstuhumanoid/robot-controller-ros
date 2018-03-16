#pragma once

#include "../DataTypes/PowerState.h"
#include "../DataTypes/PowerData.h"

class IAR60xPowerState
{
public:
    virtual bool PowerGetOnOff(PowerData::PowerSupplies supply) = 0;
    virtual PowerState::PowerSupplyState PowerGetSupplyState(PowerData::PowerSupplies supply) = 0;
};

