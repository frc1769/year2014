#include "WPILib.h"
#include "TurnControl.h"

TurnControl::TurnControl()
{
	stored_turn_value = 0.0;
}

TurnControl::~TurnControl()
{
	
}

void TurnControl::PIDWrite(float output)
{
	stored_turn_value = output;
}

float TurnControl::Value()
{
	return stored_turn_value;
}
