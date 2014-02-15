#include "WPILib.h"
#include "MecanumRobotDrive.h"
#include <math.h>
#define K_p_init 0.01
#define K_i_init 0.000013
#define K_d_init 0.005
#define DEADBAND 0.02
#define SCALE 0.3
#define EXPON 3.5

/**
 * This is a demo program showing the use of the RobotBase class.
 * The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 */ 
class RobotDemo : public IterativeRobot
{
	CANJaguar lift_ball_capture; 	  // Lifts the Capture Mechanism
	CANJaguar capture_ball; 		  // Pulls Ball In
	CANJaguar throw_ball; 			  // Moves throwing arm
	CANJaguar front_left;
	CANJaguar rear_left;
	CANJaguar front_right;
	CANJaguar rear_right;
	MecanumRobotDrive myRobot; // robot drive system
	Joystick stick; // only joystick
	Joystick other_stick; 			  // The stick for other control
	Gyro robot_angle;				  // Decides angle of robot
	PIDController control_turn;
	DigitalInput ultrasonic_in;
public:
	RobotDemo():
		lift_ball_capture(4),
		capture_ball(3),
		throw_ball(2),
		front_left(5),
		rear_left(6),
		front_right(7),
		rear_right(8),
		myRobot(front_left, rear_left, front_right, rear_right),	// these must be initialized in the same order
		stick(1),		// as they are declared above.
		other_stick(2),
		robot_angle(1),
		control_turn(K_p_init,K_i_init,K_d_init,&robot_angle,&myRobot),
		ultrasonic_in(1)
	{
		robot_angle.SetSensitivity(0.00673);
		myRobot.SetInvertedMotor (RobotDrive::kFrontRightMotor, true);
		myRobot.SetInvertedMotor (RobotDrive::kRearRightMotor, true);
		control_turn.SetSetpoint(0.0);
		control_turn.SetContinuous();
		control_turn.SetOutputRange(-1.0,1.0);
		control_turn.SetAbsoluteTolerance(0.5);
		set_angle = 0.0;
		control_turn.Enable();
		this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
	}
	
private:
	float x;
	float y;
	float z;
	float z_stick;
	double turn; 
	double ball_capture_lift_speed;   // Capture arm lift rate
	bool capture; 					  // capture button on other_stick controller
	bool uncapture; 				  // uncapture button on other_stick controller
	bool throw_it; 					  // throw button on other_stick controller
	bool un_throw_it; 	   			  // unthrow button on other_stick controller
	double capture_speed; 			  // determines capture motor speed
	double throw_speed;				  // determines throw motor speed
	double set_angle; 
	double stored_time; 
	double time_diff;
	double new_time; 
	double time_of_throw;
	double K_p;
	double K_i;
	double K_d;
	bool button_state_old;
	bool button_state_new;
	DigitalModule * dm;
	Preferences * prefs;
	I2C * prox;
	uint8_t i2c_read_val;
	Counter * m_counter;
	
/**
 * Robot-wide initialization code should go here.
 * 
 * Use this method for default Robot-wide initialization which will
 * be called when the robot is first powered on.  It will be called exactly 1 time.
 */
void RobotDemo::RobotInit() {
	bool err;
	set_angle = 0.0;
	prefs = Preferences::GetInstance();
	lift_ball_capture.SetExpiration(0.25);
	capture_ball.SetExpiration(0.25);
	throw_ball.SetExpiration(0.25);
	front_left.SetExpiration(0.25);
	rear_left.SetExpiration(0.25);
	front_right.SetExpiration(0.25);
	rear_right.SetExpiration(0.25);
	myRobot.SetExpiration(0.25);
	dm = DigitalModule::GetInstance(1);
	prox = dm->GetI2C(0x40);
	err = prox->Write(0x3,0xFE);  // Enable pin 1 of the GPIO expander to be an output
	if (err)
	{
		printf("Error in I2C write\n");
	} else
	{
		printf("No error in I2C write\n");
	}
	
	m_counter = new Counter(ultrasonic_in); // set up counter for this sensor
	m_counter->SetMaxPeriod(1.0);
	m_counter->SetSemiPeriodMode(true);
	m_counter->SetSamplesToAverage(1);
	m_counter->Reset();
	m_counter->Start();
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
	stored_time = Timer::GetPPCTimestamp();
	// Code to reset PID
	set_angle = 0.0;
	robot_angle.Reset();
	control_turn.Reset();
	control_turn.Enable();
	button_state_old = false;
	button_state_new = false;
}

/**
 * Periodic code for autonomous mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in autonomous mode.
 */
void RobotDemo::AutonomousPeriodic() {
	double entered_time;
	double time_passed;
	entered_time = Timer::GetPPCTimestamp();
	time_passed = entered_time - stored_time;
	if (time_passed < 4.0)
	{
		lift_ball_capture.Set(0.0);
		myRobot.MecanumDrive_Cartesian_Gyro_Stabilized(-0.1,-0.6,0.0);
	} else if (time_passed < 4.5)
	{
		lift_ball_capture.Set(0.8);
		myRobot.MecanumDrive_Cartesian_Gyro_Stabilized(0,0.0,0.0);
	} else 
	{
		lift_ball_capture.Set(0.0);
		myRobot.MecanumDrive_Cartesian_Gyro_Stabilized(0,0.0,0.0);
	}
}

/**
 * Initialization code for teleop mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters teleop mode.
 */
void RobotDemo::TeleopInit() {
	stored_time = Timer::GetPPCTimestamp();
	// Code to reset PID
	set_angle = 0.0;
	robot_angle.Reset();
	//control_turn.Reset();
	//control_turn.Enable();
	button_state_old = false;
	button_state_new = false;
	
}

/**
 * Periodic code for teleop mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in teleop mode.
 */
void RobotDemo::TeleopPeriodic() {
	K_p = prefs->GetFloat("K_p", 0.01);
	K_i = prefs->GetFloat("K_i", 0.000013);
	K_d = prefs->GetFloat("K_d", 0.005);

	//control_turn.SetPID(K_p, K_i, K_d);

	button_state_new = other_stick.GetRawButton(1);
/*	if (button_state_new == true && button_state_old == false)
	{
		time_of_throw = Timer::GetPPCTimestamp();
		throw_it = true;
	} else if (button_state_new = true && ((Timer::GetPPCTimestamp() - time_of_throw) < 0.05))
	{
		throw_it = true;
	} else
	{
		throw_it = false;
	}*/
	//button_state_old = button_state_new;
	throw_it = button_state_new;
	
	un_throw_it = other_stick.GetRawButton(2);
	capture = other_stick.GetRawButton(3);
	uncapture = other_stick.GetRawButton(5);
	throw_speed = throw_it ? -1.0 : (un_throw_it ? 0.4 : 0);
	capture_speed = capture ? 1.0 : (uncapture ? -1.0 : 0);
	ball_capture_lift_speed = other_stick.GetRawAxis(2);
	lift_ball_capture.Set(ball_capture_lift_speed);
	capture_ball.Set(capture_speed);
	throw_ball.Set(throw_speed);
	x = stick.GetRawAxis(1);
	y = stick.GetRawAxis(2);
	z_stick = stick.GetRawAxis(3);

	if (z_stick > DEADBAND)
	{
		z = SCALE * pow(z_stick - DEADBAND + 0.0001, EXPON) / pow(1.0 - DEADBAND + 0.0001, EXPON); 
	} 
	else if (z_stick > -DEADBAND)
	{
		z = 0.0;
	} 
	else
	{
		z = -SCALE * pow((-z_stick) - DEADBAND + 0.0001, EXPON) / pow(1.0 - DEADBAND + 0.0001, EXPON);
	}
	new_time = Timer::GetPPCTimestamp();
	time_diff = new_time - stored_time;
	if (time_diff > 0.05)
	{
		time_diff = 0.05;
	}
	//stored_time = new_time;
	//set_angle += time_diff * z * 200.0;
	//control_turn.SetSetpoint(set_angle);
	//myRobot.MecanumDrive_Cartesian_Gyro_Stabilized(x,y,0.0);
	prox->Read(0x00,1,&i2c_read_val);
	myRobot.MecanumDrive_Cartesian(x,y,z,0.0);
	SmartDashboard::PutNumber("Set Angle", set_angle);
	SmartDashboard::PutNumber("Robot Angle", robot_angle.GetAngle());
	SmartDashboard::PutNumber("Time Diff", time_diff);
	SmartDashboard::PutNumber("Proximity",i2c_read_val);
	SmartDashboard::PutNumber("Distance Info:",m_counter->GetPeriod());
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

