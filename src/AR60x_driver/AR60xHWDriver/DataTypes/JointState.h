#ifndef JOINTSTATE_H
#define JOINTSTATE_H

#include "PowerState.h"

class JointState
{
public:

	// Состояние узлов (моторов)
    enum JointStates
	{
		BRAKE, // мотор обесточен, муфта заблокирована
		STOP, // мотор включен, муфта разблокированна
		RELAX, // мотор обесточен, мефта разблокированна
		TRACE // мотор управляется углом
	};

    JointStates state;
    PowerState::PowerSupplyState supplyState;

	JointState();
	~JointState();
};

#endif // JOINTSTATE_H
