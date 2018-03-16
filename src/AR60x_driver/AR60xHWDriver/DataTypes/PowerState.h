#ifndef POWERSTATE_H
#define POWERSTATE_H

/*
* Состояние источников питания робота (напряжение и ток)
*/
class PowerState
{
public:

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

	PowerState();
	~PowerState();
};

#endif // POWERSTATE_H
