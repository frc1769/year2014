#ifndef MECANUMDRIVE_H
#define MECANUMDRIVE_H

#include "RobotDrive.h"
#include "PIDOutput.h"

class MecanumRobotDrive: public PIDOutput, public RobotDrive
{
	private:
		float stored_rotation_value;
		float stored_x_value;
		float stored_y_value;
		bool cartesian;
		float stored_magnitude;
		float stored_direction;
		float gyroAngle_stored;
	public:
		MecanumRobotDrive(SpeedController *frontLeftMotor, SpeedController *rearLeftMotor,
					SpeedController *frontRightMotor, SpeedController *rearRightMotor);
		MecanumRobotDrive(SpeedController &frontLeftMotor, SpeedController &rearLeftMotor,
					SpeedController &frontRightMotor, SpeedController &rearRightMotor);
		virtual ~MecanumRobotDrive();
		
		// PIDOutput interface
		virtual void PIDWrite(float output);
		
		void MecanumDrive_Cartesian_Gyro_Stabilized(float x, float y, float gyroAngle = 0.0);
		void MecanumDrive_Polar_Gyro_Stabilized(float magnitude, float direction);
		
		float PIDValue();
};

#endif
