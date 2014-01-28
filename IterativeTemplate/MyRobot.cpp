#include "WPILib.h"
#include "MecanumRobotDrive.h"
#define K_p_init 0.01
#define K_i_init 0.000009
#define K_d_init 0.0005

/**
 * This is a demo program showing the use of the RobotBase class.
 * The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 */ 
class RobotDemo : public IterativeRobot
{
	// Drive Motors
	CANJaguar front_left;
	CANJaguar front_right;
	CANJaguar rear_left;
	CANJaguar rear_right;
	
	CANJaguar lift_ball_capture; 	  // Lifts the Capture Mechanism
	CANJaguar capture_ball; 		  // Pulls Ball In
	CANJaguar throw_ball; 			  // Moves throwing arm
	MecanumRobotDrive robot_movement; // robot drive system
	Joystick game_pad;				  // Game pad joystick
	Joystick other_stick; 			  // The stick for other control
	Gyro robot_angle;				  // Decides angle of robot
	PIDController control_turn; 	  // Determines the desired speed of rotation
private:
	
	double x;						  // left and right speed
	double y;                         // Forward and backward speed
	double turn; 
	double ball_capture_lift_speed;   // Capture arm lift rate
	bool capture; 					  // capture button on other_stick controller
	bool uncapture; 				  // uncapture button on other_stick controller
	bool throw_it; 					  // throw button on other_stick controller
	bool un_throw_it; 	   			  // unthrow button on other_stick controller
	double capture_speed; 			  // determines capture motor speed
	double throw_speed;				  // determines throw motor speed
public:
	RobotDemo():
		front_left(5),
		front_right(7),
		rear_left(6),
		rear_right(8),
		lift_ball_capture(4),
		capture_ball(3),
		throw_ball(2),
		robot_movement
		(
			front_left, 
			rear_left, 
			front_right, 
			rear_right
		),	
		game_pad(1),		// as they are declared above.
		other_stick(2),
		robot_angle(1),
		control_turn(K_p_init,K_i_init,K_d_init,&robot_angle,&robot_movement)
	{
		robot_angle.SetSensitivity(0.00673);
		front_left.SetVoltageRampRate(100.0);
		front_right.SetVoltageRampRate(100.0);
		rear_left.SetVoltageRampRate(100.0);
		rear_right.SetVoltageRampRate(100.0);
		front_left.ConfigFaultTime(0.2);
		front_right.ConfigFaultTime(0.2);
		rear_left.ConfigFaultTime(0.2);
		rear_right.ConfigFaultTime(0.2);
		robot_movement.SetExpiration(0.2);
		robot_movement.SetInvertedMotor (RobotDrive::kFrontRightMotor, true);
		robot_movement.SetInvertedMotor (RobotDrive::kRearRightMotor, true);
		control_turn.SetSetpoint(0.0);
		control_turn.Enable();
		this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
	}
	
/**
 * Robot-wide initialization code should go here.
 * 
 * Use this method for default Robot-wide initialization which will
 * be called when the robot is first powered on.  It will be called exactly 1 time.
 */
void RobotDemo::RobotInit() {
}

/**
 * Initialization code for disabled mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters disabled mode. 
 */
void RobotDemo::DisabledInit() {
}

/**
 * Periodic code for disabled mode should go here.
 * 
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in disabled mode.
 */
void RobotDemo::DisabledPeriodic() {
}

/**
 * Initialization code for autonomous mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters autonomous mode.
 */
void RobotDemo::AutonomousInit() {
}

/**
 * Periodic code for autonomous mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in autonomous mode.
 */
void RobotDemo::AutonomousPeriodic() {
}

/**
 * Initialization code for teleop mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters teleop mode.
 */
void RobotDemo::TeleopInit() {
}

/**
 * Periodic code for teleop mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in teleop mode.
 */
void RobotDemo::TeleopPeriodic() {
	turn = 0;
	throw_it = other_stick.GetRawButton(1);
	un_throw_it = other_stick.GetRawButton(4);
	capture = other_stick.GetRawButton(2);
	uncapture = other_stick.GetRawButton(5);
	throw_speed = throw_it ? 1.0 : (un_throw_it ? -0.4 : 0);
	capture_speed = capture ? 1.0 : (uncapture ? -1.0 : 0);
	x = game_pad.GetRawAxis(3); 
	y = game_pad.GetRawAxis(4);
	ball_capture_lift_speed = other_stick.GetRawAxis(2);
	robot_movement.MecanumDrive_Cartesian_Gyro_Stabilized(x*0.9,y*0.9,0); // drive with mecanum style
	lift_ball_capture.Set(ball_capture_lift_speed);
	capture_ball.Set(capture_speed);
	throw_ball.Set(throw_speed);
}

/**
 * Initialization code for test mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters test mode.
 */
void RobotDemo::TestInit() {
}

/**
 * Periodic code for test mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in test mode.
 */
void RobotDemo::TestPeriodic() {
}

};

START_ROBOT_CLASS(RobotDemo);

