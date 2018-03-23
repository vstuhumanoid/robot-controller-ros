#ifndef JOINTSTATE_H
#define JOINTSTATE_H

#include "PowerState.h"

struct JointState
{
	/**
	 * Motor state
	 */
    enum MotorState
	{
		BRAKE,   // мотор обесточен, муфта заблокирована
		STOP,    // мотор включен, муфта разблокированна
		RELAX,   // мотор обесточен, мефта разблокированна
		TRACE    // мотор управляется углом
	} state;

	enum ControlType
	{
		POSITION_CONTROl,
		TORQUE_CONTROL
	} controlType;

    bool isBeyondLowerLimit;
	bool isBeyondUpperLimit;
};

#endif // JOINTSTATE_H
