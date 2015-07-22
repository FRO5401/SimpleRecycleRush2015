/********************************
 * TEAM 5401 - 2015
 * Programmed Dominic Altimari
********************************/
#include "WPILib.h"

// Globals {
// These variables will have the longest possible lifetime
const int IMAGE_QUALITY = 60; // percentage; scale of 0 to 100
const int MOTOR_FPS     = 200;
const int AUTO_MODE     = 3;

const double TURN_SPEED       = 0.65; // the speed at which the robot will turn at
const double MIDDLE_PRECISION = 0.6; // the speed the middle motors will run at in precision mode
const double L_R_PRECISION    = 0.20; // the speed the left and right motors will run at in precision mode
const double FEED_WHEEL_IN    = 1;
const double FEED_WHEEL_OUT   = 1;
// Globals }

class Robot : public SampleRobot{
	Joystick DriveStick;
	Joystick OpController;

	Talon  LeftDrive;
	Victor MiddleDrive;
	Talon  RightDrive;

	Victor LeftFeedWheel;
	Victor RightFeedWheel;
	Victor ArmsUp;

	DoubleSolenoid LeftLift;
	DoubleSolenoid RightLift;
	DoubleSolenoid LeftSnatcher;
	DoubleSolenoid RightSnatcher;
	DoubleSolenoid Arms;
	DoubleSolenoid LeftContainerLift;
	DoubleSolenoid RightContainerLift;

public:
	Robot() :
		DriveStick  (0),
		OpController(1),

		LeftDrive     (1),
		MiddleDrive   (2), // Middle should be on 1
		RightDrive    (0), //  Right should be on 2
		LeftFeedWheel (3),
		RightFeedWheel(4),
		ArmsUp        (8),

		LeftLift     	  (0, 0, 1),
		RightLift   	  (0, 2, 3),
		LeftSnatcher 	  (0, 4, 5),
		RightSnatcher	  (0, 6, 7),
		Arms       		  (1, 0, 1),
		LeftContainerLift (1, 2, 3),
		RightContainerLift(1, 4, 5)
	{
	}

	void SetMotors(double LeftDriveDesired, double MiddleDriveDesired, double RightDriveDesired){
		LeftDrive  .Set(LeftDriveDesired);
		MiddleDrive.Set(MiddleDriveDesired);
		RightDrive .Set(RightDriveDesired);
	}

	void RobotInit() override{
		CameraServer::GetInstance()->SetQuality(IMAGE_QUALITY);
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");

//		AUTO_MODE = SmartDashboard::GetNumber("AutoMode");
//		DriverStation::ReportError(std::to_string(AUTO_MODE));
	}

	void Autonomous(){
		LeftContainerLift .Set(DoubleSolenoid::kReverse);
		RightContainerLift.Set(DoubleSolenoid::kReverse);

		if(AUTO_MODE == 1){
			Wait(1);
			RightDrive.Set( 0.47);
			LeftDrive .Set(-0.4);
			Wait(2);
			MiddleDrive.Set(-0.6);
			Wait(0.5);
			MiddleDrive.Set(0);
			Wait(2);
			RightDrive.Set(0.35);
			LeftDrive .Set(-0.3);
			Wait(0.8);
			RightDrive.Set(0);
			LeftDrive .Set(0);
			MiddleDrive.Set(-0.6);
			Wait(1);
			MiddleDrive.Set(0);
		}else if(AUTO_MODE == 2){
			Wait(1);
			RightDrive.Set( 0.44);
			LeftDrive .Set(-0.4);
			Wait(4);
			RightDrive.Set(0.33);
			LeftDrive .Set(-0.3);
			Wait(0.6);
			RightDrive.Set(0);
			LeftDrive .Set(0);
		}else if(AUTO_MODE == 3){
			RightDrive.Set(-0.25);
			LeftDrive .Set(0.27);
			Wait(0.125);
			LeftSnatcher .Set(DoubleSolenoid::kForward);
			RightSnatcher.Set(DoubleSolenoid::kForward);
			Wait(1);
/*			RightDrive.Set(0.25);
			LeftDrive .Set(-0.28);
			Wait(0.3);
			RightDrive.Set(-0.25);
			LeftDrive .Set(0.28);
			Wait(0.3);
*/
			RightDrive.Set( 1);
			LeftDrive .Set(-1);
			Wait(1.125);
			LeftSnatcher .Set(DoubleSolenoid::kReverse);
			RightSnatcher.Set(DoubleSolenoid::kReverse);
			RightDrive.Set(0);
			LeftDrive .Set(0);
		}else if(AUTO_MODE == 4){
			RightDrive.Set(0.33);
			LeftDrive .Set(-0.3);
			Wait(2);
			RightDrive.Set(0);
			LeftDrive .Set(0);
		}
	}

	void OperatorControl(){
		while(IsOperatorControl() && IsEnabled()){
			double LeftDriveDesired;
			double MiddleDriveDesired;
			double RightDriveDesired;

			double Slew     = DriveStick.GetRawAxis(0);
			double Throttle = DriveStick.GetRawAxis(1);
			double Twist    = DriveStick.GetRawAxis(2);

			if(abs(Throttle) < 0.2){
				Twist = Twist * TURN_SPEED;
			}

			if(DriveStick.GetRawButton(1)){ // brake mode
				LeftDriveDesired   = 0;
				MiddleDriveDesired = 0;
				RightDriveDesired  = 0;
			}else{
				LeftDriveDesired   = Twist - Throttle;
				MiddleDriveDesired = Slew;
				RightDriveDesired  = Twist + Throttle;
			}

			if(DriveStick.GetRawButton(2)){ // precision mode
				LeftDriveDesired   = LeftDriveDesired   * L_R_PRECISION;
				MiddleDriveDesired = MiddleDriveDesired * MIDDLE_PRECISION; // too low will result in no movement
				RightDriveDesired  = RightDriveDesired  * L_R_PRECISION;
			}

			SetMotors(LeftDriveDesired, MiddleDriveDesired, RightDriveDesired);

			if(OpController.GetRawButton(1)){
				LeftLift .Set(DoubleSolenoid::kForward);
				RightLift.Set(DoubleSolenoid::kForward);
			}else if(OpController.GetRawButton(4)){
				LeftLift .Set(DoubleSolenoid::kReverse);
				RightLift.Set(DoubleSolenoid::kReverse);
			}

			if(OpController.GetRawButton(2)){
				LeftContainerLift .Set(DoubleSolenoid::kForward);
				RightContainerLift.Set(DoubleSolenoid::kForward);
			}else if(OpController.GetRawButton(3)){
				LeftContainerLift .Set(DoubleSolenoid::kReverse);
				RightContainerLift.Set(DoubleSolenoid::kReverse);

			}

			if(OpController.GetRawButton(5)){
				LeftFeedWheel.Set(FEED_WHEEL_IN);
			}else{
				LeftFeedWheel.Set(-OpController.GetRawAxis(2) * FEED_WHEEL_OUT);
			}

			if(OpController.GetRawButton(6)){
				RightFeedWheel.Set(-FEED_WHEEL_IN);
			}else{
				RightFeedWheel.Set(OpController.GetRawAxis(3) * FEED_WHEEL_OUT);
			}

			if(OpController.GetRawButton(9)){
				Arms.Set(DoubleSolenoid::kForward);
			}else{
				Arms.Set(DoubleSolenoid::kReverse);
			}

			if(OpController.GetRawAxis(1) > 0.2){
				ArmsUp.Set(0.5);
			}else if(OpController.GetRawAxis(1) < -0.2){
				ArmsUp.Set(-0.5);
			}else{
				ArmsUp.Set(0);
			}

			if(OpController.GetPOV() == 0){
				LeftSnatcher .Set(DoubleSolenoid::kReverse);
				RightSnatcher.Set(DoubleSolenoid::kReverse);
			}else if(OpController.GetPOV() == 180){
				LeftSnatcher .Set(DoubleSolenoid::kForward);
				RightSnatcher.Set(DoubleSolenoid::kForward);
			}

			Wait(1 / MOTOR_FPS);
		}
	}
};

START_ROBOT_CLASS(Robot);
