#ifndef POWERSTATE_H
#define POWERSTATE_H

/*
* Состояние источников питания робота (напряжение и ток)
*/
struct PowerState
{
    struct PowerSupplyState
    {
		float Voltage;
		float Current;
    };

	PowerSupplyState power12VState;
	PowerSupplyState power6V1State;
	PowerSupplyState power6V2State;
	PowerSupplyState power8V1State;
	PowerSupplyState power8V2State;
	PowerSupplyState power48VState;
};

#endif // POWERSTATE_H
