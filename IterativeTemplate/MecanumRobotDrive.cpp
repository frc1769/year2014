#include "MecanumRobotDrive.h"

#include "CANJaguar.h"
#include "GenericHID.h"
#include "Joystick.h"
#include "Jaguar.h"
#include "NetworkCommunication/UsageReporting.h"
#include "Utility.h"
#include "WPIErrors.h"
#include <math.h>

#define max(x, y) (((x) > (y)) ? (x) : (y))

MecanumRobotDrive::MecanumRobotDrive(SpeedController &frontLeftMotor, SpeedController &rearLeftMotor,
			SpeedController &frontRightMotor, SpeedController &rearRightMotor) : 
			RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor)
{

}

/**
 * Drive method for Mecanum wheeled robots.
 *
 * A method for driving with Mecanum wheeled robots. There are 4 wheels
 * on the robot, arranged so that the front and back wheels are toed in 45 degrees.
 * When looking at the wheels from the top, the roller axles should form an X across the robot.
 * 
 * This is designed to be directly driven by joystick axes.
 *
 * @param x The speed that the robot should drive in the X direction. [-1.0..1.0]
 * @param y The speed that the robot should drive in the Y direction.
 * This input is inverted to match the forward == -1.0 that joysticks produce. [-1.0..1.0]
 * @param gyroAngle The current angle reading from the gyro.  Use this to implement field-oriented controls.
 */
void MecanumRobotDrive::MecanumDrive_Cartesian_Gyro_Stabilized(float x, float y, float gyroAngle)
{
	stored_x_value = x;
	stored_y_value = y;
	cartesian = true;
	gyroAngle_stored = gyroAngle;

	double xIn = x;
	double yIn = y;
	// Negate y for the joystick.
	yIn = -yIn;
	// Compenstate for gyro angle.
	RotateVector(xIn, yIn, gyroAngle);

	double wheelSpeeds[kMaxNumberOfMotors];
	wheelSpeeds[kFrontLeftMotor] = xIn + yIn + stored_rotation_value;
	wheelSpeeds[kFrontRightMotor] = -xIn + yIn - stored_rotation_value;
	wheelSpeeds[kRearLeftMotor] = -xIn + yIn + stored_rotation_value;
	wheelSpeeds[kRearRightMotor] = xIn + yIn - stored_rotation_value;

	Normalize(wheelSpeeds);

	uint8_t syncGroup = 0x80;

	m_frontLeftMotor->Set(wheelSpeeds[kFrontLeftMotor] * m_invertedMotors[kFrontLeftMotor] * m_maxOutput, syncGroup);
	m_frontRightMotor->Set(wheelSpeeds[kFrontRightMotor] * m_invertedMotors[kFrontRightMotor] * m_maxOutput, syncGroup);
	m_rearLeftMotor->Set(wheelSpeeds[kRearLeftMotor] * m_invertedMotors[kRearLeftMotor] * m_maxOutput, syncGroup);
	m_rearRightMotor->Set(wheelSpeeds[kRearRightMotor] * m_invertedMotors[kRearRightMotor] * m_maxOutput, syncGroup);

	CANJaguar::UpdateSyncGroup(syncGroup);
	
	m_safetyHelper->Feed();
}


/**
 * Drive method for Mecanum wheeled robots.
 *
 * A method for driving with Mecanum wheeled robots. There are 4 wheels
 * on the robot, arranged so that the front and back wheels are toed in 45 degrees.
 * When looking at the wheels from the top, the roller axles should form an X across the robot.
 *
 * @param magnitude The speed that the robot should drive in a given direction. [-1.0..1.0]
 * @param direction The direction the robot should drive in degrees. The direction and maginitute are
 * independent of the rotation rate.
 */
void MecanumRobotDrive::MecanumDrive_Polar_Gyro_Stabilized(float magnitude, float direction)
{
	cartesian = false;
	stored_magnitude = magnitude;
	stored_direction = direction;

	// Normalized for full power along the Cartesian axes.
	magnitude = Limit(magnitude) * sqrt(2.0);
	// The rollers are at 45 degree angles.
	double dirInRad = (direction + 45.0) * 3.14159 / 180.0;
	double cosD = cos(dirInRad);
	double sinD = sin(dirInRad);

	double wheelSpeeds[kMaxNumberOfMotors];
	wheelSpeeds[kFrontLeftMotor] = sinD * magnitude + stored_rotation_value;
	wheelSpeeds[kFrontRightMotor] = cosD * magnitude - stored_rotation_value;
	wheelSpeeds[kRearLeftMotor] = cosD * magnitude + stored_rotation_value;
	wheelSpeeds[kRearRightMotor] = sinD * magnitude - stored_rotation_value;

	Normalize(wheelSpeeds);

	uint8_t syncGroup = 0x80;

	m_frontLeftMotor->Set(wheelSpeeds[kFrontLeftMotor] * m_invertedMotors[kFrontLeftMotor] * m_maxOutput, syncGroup);
	m_frontRightMotor->Set(wheelSpeeds[kFrontRightMotor] * m_invertedMotors[kFrontRightMotor] * m_maxOutput, syncGroup);
	m_rearLeftMotor->Set(wheelSpeeds[kRearLeftMotor] * m_invertedMotors[kRearLeftMotor] * m_maxOutput, syncGroup);
	m_rearRightMotor->Set(wheelSpeeds[kRearRightMotor] * m_invertedMotors[kRearRightMotor] * m_maxOutput, syncGroup);

	CANJaguar::UpdateSyncGroup(syncGroup);
	
	m_safetyHelper->Feed();
}



void MecanumRobotDrive::PIDWrite(float output)
{
	stored_rotation_value = output;
	double wheelSpeeds[kMaxNumberOfMotors];
	if (cartesian)
	{
		double xIn = stored_x_value;
		double yIn = stored_y_value;
		// Negate y for the joystick.
		yIn = -yIn;
		// Compenstate for gyro angle.
		RotateVector(xIn, yIn, gyroAngle_stored);

		wheelSpeeds[kFrontLeftMotor] = xIn + yIn + stored_rotation_value;
		wheelSpeeds[kFrontRightMotor] = -xIn + yIn - stored_rotation_value;
		wheelSpeeds[kRearLeftMotor] = -xIn + yIn + stored_rotation_value;
		wheelSpeeds[kRearRightMotor] = xIn + yIn - stored_rotation_value;
	} 
	else
	{

		// Normalized for full power along the Cartesian axes.
		float magnitude = Limit(stored_magnitude) * sqrt(2.0);
		// The rollers are at 45 degree angles.
		double dirInRad = (stored_direction + 45.0) * 3.14159 / 180.0;
		double cosD = cos(dirInRad);
		double sinD = sin(dirInRad);

		wheelSpeeds[kFrontLeftMotor] = sinD * magnitude + stored_rotation_value;
		wheelSpeeds[kFrontRightMotor] = cosD * magnitude - stored_rotation_value;
		wheelSpeeds[kRearLeftMotor] = cosD * magnitude + stored_rotation_value;
		wheelSpeeds[kRearRightMotor] = sinD * magnitude - stored_rotation_value;
	}
	
	Normalize(wheelSpeeds);

	uint8_t syncGroup = 0x80;

	m_frontLeftMotor->Set(wheelSpeeds[kFrontLeftMotor] * m_invertedMotors[kFrontLeftMotor] * m_maxOutput, syncGroup);
	m_frontRightMotor->Set(wheelSpeeds[kFrontRightMotor] * m_invertedMotors[kFrontRightMotor] * m_maxOutput, syncGroup);
	m_rearLeftMotor->Set(wheelSpeeds[kRearLeftMotor] * m_invertedMotors[kRearLeftMotor] * m_maxOutput, syncGroup);
	m_rearRightMotor->Set(wheelSpeeds[kRearRightMotor] * m_invertedMotors[kRearRightMotor] * m_maxOutput, syncGroup);

	CANJaguar::UpdateSyncGroup(syncGroup);
}
