#include "WPILib.h"
#include <jaguar.h>
#include <math.h>
#include <Timer.h>

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	
	CANJaguar fl;
	CANJaguar fr;
	CANJaguar rl;
	CANJaguar rr;
	CANJaguar shooter;
	CANJaguar loader;
	CANJaguar thrower;
	RobotDrive myRobot; // robot drive system
	Joystick stick; // Controller
	Joystick stick_climb; // Joysitck for Climber
	AnalogChannel temperature;
	NetworkTable *table;
	Gyro robotangle;
	DigitalInput loadsw;
	DigitalInput rpm_ir;
	Counter cnt;
	Timer tmr;
	Timer autotimer;
private:
	double K_p;
	double K_i;
	double K_d;
	double x;
	double y;
	double turn;
	double current_angle;
	double current_time;
	double old_time;
	double new_rpm_time;
	double old_rpm_time;
	double current_angle_centered;
	double error_angle;
	double old_error_angle;
	double integral_angle;
	double derivitive_angle;
	double desired_angle;
	bool first_time;
	double turnx;
	double turny;
	double turnmagnitude;
	bool turnactive;
	double turnangle; 
	double storedangle;
	double startangle;
	bool loadsw_old;
	bool runloader;
	double loadertime;
	double old_rpm_count;
	double new_rpm_count;
	double rpm;
	bool first_shoot;
	bool second_shoot;
	bool third_shoot;
	double starting_position;
	/* front right goes to 1
	 * back right goes to 2
	 * front left goes to 3
	 * back left goes to 4
	 */
public:
	RobotDemo(void):
		fl(3),
		fr(14),
		rl(4),
		rr(2),
		shooter(5),
		loader(6), 
		thrower(7),
		myRobot(fl, rl, fr, rr),	// these must be initialized in the same order
		stick(1),		// as they are declared above.
        stick_climb(2),
		temperature(2),
        robotangle(1),
        loadsw(1),
        rpm_ir(2),
        cnt(2),
        tmr(),
        autotimer()
	{
		
		robotangle.SetSensitivity(0.0067303055555556);
		loadsw_old = false;
		runloader = false;
		K_p = 0.01;
		K_i = 0.000009;
		K_d = 0.0005;
		fl.SetVoltageRampRate(100.0);
		fr.SetVoltageRampRate(100.0);
		rl.SetVoltageRampRate(100.0);
		rr.SetVoltageRampRate(100.0);
		fl.ConfigFaultTime(0.2);
		fr.ConfigFaultTime(0.2);
		rl.ConfigFaultTime(0.2);
		rr.ConfigFaultTime(0.2);
		shooter.SetVoltageRampRate(100.0);
		loader.SetVoltageRampRate(100.0);
		thrower.SetVoltageRampRate(100.0);
		myRobot.SetExpiration(0.2);
		myRobot.SetInvertedMotor (RobotDrive::kFrontRightMotor, true);
		myRobot.SetInvertedMotor (RobotDrive::kRearRightMotor, true);
		table = NetworkTable::GetTable("datatable");
		//table->PutNumber("temperature",temperature.GetVoltage());
		//table->PutNumber("robotangle",robotangle.GetAngle());
		first_time = true;
		startangle = robotangle.GetAngle();
		table->PutNumber("K_i",K_i);
		table->PutNumber("K_p",K_p);
		table->PutNumber("K_d",K_d);
		cnt.SetUpSource(2);
		cnt.ClearDownSource();
		cnt.Start();
		old_rpm_count = cnt.Get();
		old_rpm_time = Timer::GetFPGATimestamp();
		tmr.Start();
		tmr.Reset();
		first_shoot = true;
		second_shoot = true;
		third_shoot = true;
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		myRobot.SetSafetyEnabled(true);
		autotimer.Reset();
		autotimer.Start();
		current_angle = robotangle.GetAngle();
		current_time = Timer::GetFPGATimestamp();
		current_angle_centered = (fmod((current_angle+180.0),360.0)-180);
		startangle = current_angle_centered;
		starting_position = stick_climb.GetRawAxis(3);
		while (IsAutonomous())
		{
			if (IsDisabled())
			{
				autotimer.Reset();
				autotimer.Start();
				first_shoot = true;
				second_shoot = true;
				third_shoot = true;
				current_angle = robotangle.GetAngle();
				current_time = Timer::GetFPGATimestamp();
				current_angle_centered = (fmod((current_angle+180.0),360.0)-180);
				startangle = current_angle_centered;
				starting_position = stick_climb.GetRawAxis(3);
			} 
			else
			{
				new_rpm_time = tmr.Get();
				if (new_rpm_time > 0.25)
				{
					new_rpm_count = cnt.Get();
					rpm = (new_rpm_count - old_rpm_count)/(new_rpm_time);
					
					table->PutNumber("rpm",rpm);
					tmr.Reset();
					old_rpm_count = new_rpm_count;
				}
				
				K_i = table->GetNumber("K_i");
				K_p = table->GetNumber("K_p");
				K_d = table->GetNumber("K_d");
				
				x = 0;
				y = 0;
				if (starting_position>0.5)
				{
					if (autotimer.Get() < 1.0)
					{
						x = 0;
						y = 0;
						turnangle = 0.0;
					}
				    else if (autotimer.Get() < 1.65)
					{
						x = 1.0;
						
						turnangle = 0.0;
					}
					else if (autotimer.Get() < 3)
					{
						x = 0.0;
						
						turnangle = 0.0;
						shooter.Set(1.0);
					}
					else if (autotimer.Get() < 3.75)
					{
						x = 0.0;
						y = -1.0;
					}
					else if (autotimer.Get() < 5)
					{
						turnangle = -42.0;
					}
					else if (autotimer.Get() < 7)
					{
						if (rpm > 380 && first_shoot)
						{
							runloader = true ;
							loadertime = Timer::GetFPGATimestamp();
							first_shoot = false;
						}
					}
					else if (autotimer.Get() < 9)
					{
						if (rpm > 380 && second_shoot)
						{
							runloader = true ;
							loadertime = Timer::GetFPGATimestamp();
							second_shoot = false;
						}
					}
					else if (autotimer.Get() < 11)
					{
						turnangle = 90.0;
					}
					else if (autotimer.Get() < 12)
					{
						turnangle = -42.0;
					}
					else if (autotimer.Get() < 13)
					{
						if (rpm > 380 && third_shoot)
						{
							runloader = true ;
							loadertime = Timer::GetFPGATimestamp();
							third_shoot = false;
						}
					}
					else
					{
						runloader = false;
						loadertime = Timer::GetFPGATimestamp();
						shooter.Set(0.0);
					}
				} 
				else if (starting_position>0)
				{
					if (autotimer.Get() < 1.0)
					{
						x = 0;
						y = 0;
						turnangle = 0.0;
					}
					else if (autotimer.Get() < 1.4)
					{
						x = 1.0;
						
						turnangle = 0.0;
					}
					else if (autotimer.Get() < 3)
					{
						x = 0.0;
						
						turnangle = 0.0;
						shooter.Set(1.0);
					}
					else if (autotimer.Get() < 4.1)
					{
						x = 0.0;
						y = -1.0;
					}
					else if (autotimer.Get() < 5)
					{
						turnangle = -42.0;
					}
					else if (autotimer.Get() < 8)
					{
						if (rpm > 380 && first_shoot)
						{
							runloader = true ;
							loadertime = Timer::GetFPGATimestamp();
							first_shoot = false;
						}
					}
					else if (autotimer.Get() < 11)
					{
						if (rpm > 380 && second_shoot)
						{
							runloader = true ;
							loadertime = Timer::GetFPGATimestamp();
							second_shoot = false;
						}
					}
					else if (autotimer.Get() < 12.5)
					{
						turnangle = 90.0;
					}
					else if (autotimer.Get() < 13)
					{
						turnangle = -42.0;
					}
					else if (autotimer.Get() < 14)
					{
						if (rpm > 380 && third_shoot)
						{
							runloader = true ;
							loadertime = Timer::GetFPGATimestamp();
							third_shoot = false;
						}
					}
					else
					{
						runloader = false;
						loadertime = Timer::GetFPGATimestamp();
						shooter.Set(0.0);
					}
				} 
				else if (starting_position > -0.5)
				{
					if (autotimer.Get() < 1.0)
					{
						x = 0;
						y = 0;
						turnangle = 0.0;
					}
					else if (autotimer.Get() < 1.4)
					{
						x = -1.0;
						
						turnangle = 0.0;
					}
					else if (autotimer.Get() < 3)
					{
						x = 0.0;
						
						turnangle = 0.0;
						shooter.Set(1.0);
					}
					else if (autotimer.Get() < 4.1)
					{
						x = 0.0;
						y = -1.0;
					}
					else if (autotimer.Get() < 5)
					{
						turnangle = 42.0;
					}
					else if (autotimer.Get() < 8)
					{
						if (rpm > 380 && first_shoot)
						{
							runloader = true ;
							loadertime = Timer::GetFPGATimestamp();
							first_shoot = false;
						}
					}
					else if (autotimer.Get() < 11)
					{
						if (rpm > 380 && second_shoot)
						{
							runloader = true ;
							loadertime = Timer::GetFPGATimestamp();
							second_shoot = false;
						}
					}
					else if (autotimer.Get() < 12.5)
					{
						turnangle = -90.0;
					}
					else if (autotimer.Get() < 13)
					{
						turnangle = 42.0;
					}
					else if (autotimer.Get() < 14)
					{
						if (rpm > 380 && third_shoot)
						{
							runloader = true ;
							loadertime = Timer::GetFPGATimestamp();
							third_shoot = false;
						}
					}
					else
					{
						runloader = false;
						loadertime = Timer::GetFPGATimestamp();
						shooter.Set(0.0);
					}
				} 
				else 
				{
					if (autotimer.Get() < 0.5)
					{
						x = 0.0;
						y = -0.8;
						turnangle = 0.0;
					}
					else if (autotimer.Get() < 3.0)
					{
						x = 0.0;
						y = 0.0;
						turnangle = 0.0;
						shooter.Set(1.0);
					}
					else if (autotimer.Get() < 4)
					{
						if (rpm > 380 && first_shoot)
						{
							runloader = true ;
							loadertime = Timer::GetFPGATimestamp();
							first_shoot = false;
						}
					}
					else if (autotimer.Get() < 6)
					{
						if (rpm > 380 && second_shoot)
						{
							runloader = true ;
							loadertime = Timer::GetFPGATimestamp();
							second_shoot = false;
						}
					}
					else if (autotimer.Get() < 8)
					{
						if (rpm > 380 && third_shoot)
						{
							runloader = true ;
							loadertime = Timer::GetFPGATimestamp();
							third_shoot = false;
						}
					}
					else
					{
						runloader = false;
						loadertime = Timer::GetFPGATimestamp();
						shooter.Set(0.0);
					}
				}
				
				desired_angle = turnangle + startangle;
		
				current_angle = robotangle.GetAngle();
				current_time = Timer::GetFPGATimestamp();
				current_angle_centered = (fmod((current_angle+180.0),360.0)-180);
			
				//desired_angle = 0;
				error_angle = (fmod((desired_angle - current_angle_centered + 180.0) , 360.0) - 180);
				if (not first_time)
				{
					integral_angle = integral_angle + error_angle * (current_time - old_time);
					derivitive_angle = (error_angle - old_error_angle)/(current_time - old_time); 
					turn =  error_angle * K_p + integral_angle * K_i + derivitive_angle * K_d;
				} 
				else
				{
					turn = 0;
					integral_angle = 0;
					derivitive_angle = 0;
				}
				
				old_time = current_time;
				old_error_angle = error_angle;
				first_time = false;
				
				if (turn < -1) 
				{
					turn = -1;
				} 
				else if (turn > 1)
				{
					turn = 1;
				}
				
				myRobot.MecanumDrive_Cartesian(x*0.9,y*0.9,turn,0);
				table->PutNumber("turn_value",turn);
				Wait(0.005);				// wait for a motor update time
				table->PutNumber("temperature",temperature.GetVoltage());
				table->PutNumber("robotangle",robotangle.GetAngle());
				table->PutBoolean("switch",loadsw.Get());
				loadsw_old <= loadsw.Get();
	
				if(loadsw.Get() && !(loadsw_old)&& (Timer::GetFPGATimestamp()-loadertime>.25))
				{
					runloader = false;
				
				}
				
				if(runloader)
				{
					loader.Set(-1);
				
				}else
				{
					loader.Set(0);
				}
			}
		}
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		myRobot.SetSafetyEnabled(true);
		current_angle = robotangle.GetAngle();
		current_time = Timer::GetFPGATimestamp();
		current_angle_centered = (fmod((current_angle+180.0),360.0)-180);
		desired_angle = current_angle_centered;
		startangle = current_angle_centered;
		runloader = false;
		first_shoot = true;
		second_shoot = true;
		third_shoot = true;
		while (IsOperatorControl())
		{
			if (IsEnabled())
			{
				new_rpm_time = tmr.Get();
				if (new_rpm_time > 0.25)
				{
					new_rpm_count = cnt.Get();
					rpm = (new_rpm_count - old_rpm_count)/(new_rpm_time);
					
					table->PutNumber("rpm",rpm);
					tmr.Reset();
					old_rpm_count = new_rpm_count;
				}
				
				
				
				
				K_i = table->GetNumber("K_i");
				K_p = table->GetNumber("K_p");
				K_d = table->GetNumber("K_d");
				
				x = stick.GetRawAxis(3); 
				y = stick.GetRawAxis(4);
				turnx = stick.GetRawAxis(1);
				turny = stick.GetRawAxis(2) ;
	
				turnmagnitude = sqrt(pow(turnx,2) + pow(turny,2));
				turnangle = atan2(turny, turnx)/3.14159*180;
				current_angle = robotangle.GetAngle();
				current_time = Timer::GetFPGATimestamp();
				current_angle_centered = (fmod((current_angle+180.0),360.0)-180);
				if (turnmagnitude > 0.75)
				{
					if (turnactive)
					{
						desired_angle = turnangle - storedangle + startangle;
					}
					else
					{
						turnactive = true;
						storedangle = turnangle;
						desired_angle = current_angle_centered;
						startangle = current_angle_centered;
					}
				
				}
				else
				{
					if (turnactive)
					{
						turnactive = false; 
						desired_angle = current_angle_centered;	
						startangle = current_angle_centered;
					}
					
				}
			
				//desired_angle = 0;
				error_angle = (fmod((desired_angle - current_angle_centered + 180.0) , 360.0) - 180);
				if (not first_time)
				{
					integral_angle = integral_angle + error_angle * (current_time - old_time);
					derivitive_angle = (error_angle - old_error_angle)/(current_time - old_time); 
					turn =  error_angle * K_p + integral_angle * K_i + derivitive_angle * K_d;
				} 
				else
				{
					turn = 0;
					integral_angle = 0;
					derivitive_angle = 0;
				}
				
				old_time = current_time;
				old_error_angle = error_angle;
				first_time = false;
				
				if (turn < -1) 
				{
					turn = -1;
				} 
				else if (turn > 1)
				{
					turn = 1;
				}
				
				myRobot.MecanumDrive_Cartesian(x*0.9,y*0.9,turn,0);
				table->PutNumber("turn_value",turn);
				Wait(0.005);				// wait for a motor update time
				table->PutNumber("temperature",temperature.GetVoltage());
				table->PutNumber("robotangle",robotangle.GetAngle());
				table->PutBoolean("switch",loadsw.Get());
				loadsw_old <= loadsw.Get();
				if(stick.GetRawButton(2))
				{
					thrower.Set(1);
				} 
				else
				{
					thrower.Set(0);
				}
				
				if(stick.GetRawButton(1))
				{
					runloader = true ;
					loadertime = Timer::GetFPGATimestamp();
				}
				else if(loadsw.Get() && !(loadsw_old)&& (Timer::GetFPGATimestamp()-loadertime>.25))
				{
					runloader = false;
				
				}
				
				if(runloader)
				{
					loader.Set(-1);
				
				}else
				{
					loader.Set(0);
				}
			// 	thrower.Set(stick_climb.GetRawAxis(2)); 
				
			} 
			if (IsDisabled())
			{
				robotangle.SetSensitivity(0.0067303055555556);
				loadsw_old = false;
				runloader = false;
				K_p = 0.01;
				K_i = 0.000009;
				K_d = 0.0005;
				fl.SetVoltageRampRate(100.0);
				fr.SetVoltageRampRate(100.0);
				rl.SetVoltageRampRate(100.0);
				rr.SetVoltageRampRate(100.0);
				fl.ConfigFaultTime(0.1);
				fr.ConfigFaultTime(0.1);
				rl.ConfigFaultTime(0.1);
				rr.ConfigFaultTime(0.1);
				shooter.SetVoltageRampRate(100.0);
				loader.SetVoltageRampRate(100.0);
				thrower.SetVoltageRampRate(100.0);
				myRobot.SetExpiration(0.1);
				myRobot.SetInvertedMotor (RobotDrive::kFrontRightMotor, true);
				myRobot.SetInvertedMotor (RobotDrive::kRearRightMotor, true);
				table = NetworkTable::GetTable("datatable");
				first_time = true;
				table->PutNumber("K_i",K_i);
				table->PutNumber("K_p",K_p);
				table->PutNumber("K_d",K_d);
				cnt.SetUpSource(2);
				cnt.ClearDownSource();
				cnt.Start();
				old_rpm_count = cnt.Get();
				old_rpm_time = Timer::GetFPGATimestamp();
				tmr.Start();
				tmr.Reset();
				myRobot.SetSafetyEnabled(true);
				current_angle = robotangle.GetAngle();
				current_time = Timer::GetFPGATimestamp();
				current_angle_centered = (fmod((current_angle+180.0),360.0)-180);
				desired_angle = current_angle_centered;
				startangle = current_angle_centered;
				runloader = false;
			}
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);

