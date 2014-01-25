#ifndef PID_TURN_H
#define PID_TURN_H

#include "PIDOutput.h"

class TurnControl : public PIDOutput
{
	private:
		float stored_turn_value;
	public:
		explicit TurnControl();
		virtual ~TurnControl();
		
		// PIDOutput interface
		virtual void PIDWrite(float output);
		
		float Value();
};

#endif
