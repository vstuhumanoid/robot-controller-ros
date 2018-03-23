#pragma once
#include <DataTypes/PowerData.h>

class IAR60xPowerControl
{
public:
    virtual PowerState::PowerSupplyState JointGetSupplyState(uint8_t joint) = 0;
    virtual PowerState::PowerSupplyState PowerGetSupplyState(PowerData::PowerSupplies supply) = 0;
    virtual void SupplySetOnOff(PowerData::PowerSupplies supply, bool onOffState) = 0;
    virtual bool SupplyGetOnOff(PowerData::PowerSupplies supply) = 0;
};

