#include <iostream>
#include <iomanip>
#include "WPILib.h"
#include "PixyTracker.hpp"
#include "AHRS.h"
#include <CANTalon.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot: public IterativeRobot {
private:

	Servo *gearServoRight;
	Servo *gearServoLeft;
	int timer = 0;
	int autoTimer = 0;
	TalonSRX *climber;
	double driveAngle;
	bool isRobotCentric = false;
    double last_world_linear_accel_x = 0.0;
    double last_world_linear_accel_y = 0.0;

	const std::string autoNameDefault = "center";
	std::string autoSelected;

	// Servo constants
	const double kServoRightOpen   = 0.3; //0.5;
	const double kServoRightClosed = 0.0; //0.9; //-0.5;
	const double kServoLeftOpen    = 0.0; //-0.1;
	const double kServoLeftClosed  = 0.3; //0.5;


	// Tunable parameters for target detection
	const int kTrackingBrightness = 22;
	const int kNormalBrightness   = 128;
	const int kGearTargetWidth    = 20;
	const int kGearTargetHeight   = 50;
	const int kGearTargetArea = (kGearTargetWidth * kGearTargetHeight);

	// Tunable parameters for driving
	const double kSpinRateLimiter = 0.5;

    int autoDriveFowardTime    = 0;
	double autoTurnToGearAngle = 0.0;

	// Tunable parameters for autonomous
	const double kAutoSpeed               = 0.5;
    const double kCollisionThresholdDelta = 0.5;
    const int kTurningToGearTime          = 80;
    const int kDriveToGearMaxTime         = 600;
    const int kPlaceGearTime              = 50;
    const int kDriveBackwardTime          = 50;

	// Tunable parameters for the Strafe PID Controller
	const double ksP = 0.01;
	const double ksI = 0.00;
	const double ksD = 0.03;
	const double ksF = 0.00;

	// Tunable parameters for the Turn PID Controller
	const double kP = 0.01;
	const double kI = 0.00;
	const double kD = 0.03;
	const double kF = 0.00;

	/* This tuning parameter indicates how close to "on target" the    */
	/* PID Controller will attempt to get.                             */
	const double kToleranceDegrees = 1.0;
	const double kToleranceStrafe = 2.0;

	enum AutoState {
		kDoNothing,
		kDrivingForward,
		kPlacingGear,
		kDrivingBackward,
		kTurningToGear,
		kDrivingToGear
	} autoState = kDoNothing;

	PixyTracker *m_pixy;
	int m_signature = 1;
	PixyTracker::Target m_targets[2];

	RobotDrive *drive;
	TalonSRX   *lFrontMotor;
	TalonSRX   *lBackMotor;
	TalonSRX   *rFrontMotor;
	TalonSRX   *rBackMotor;
	Joystick   *stick;
	AHRS       *ahrs; // navX MXP

	PIDController *turnController;      // PID Controller
	PIDController *strafeController;    // PID Controller

	// Output from the Turn (Angle) PID Controller
	class TurnPIDOutput : public PIDOutput {
	public:
		double correction;
		void PIDWrite(double output) {
			correction = output;
		}
	} turnPIDOutput;

	// Output from the Strafe PID Controller
	class StrafePIDOutput : public PIDOutput {
	public:
		double correction;
		void PIDWrite(double output) {
			correction = output;
		}
	} strafePIDOutput;

	// Data source for the Strafe PID Controller
	class StrafePIDSource : public PIDSource {
	public:
		StrafePIDSource() : offset(0.0) {}
		double PIDGet() {
			return offset;
		}
		void calcTargetOffset(PixyTracker::Target* targets, int count) {
			if (count < 2) {
				offset = 0.0;
			} else {
                //if ((targets[0].block.width > kGearTargetWidth)   &&
				//	(targets[0].block.height > kGearTargetHeight) &&
 				//	(targets[1].block.width > kGearTargetWidth)   &&
 				//	(targets[1].block.height > kGearTargetHeight) &&
 				//	(abs(targets[0].block.y - targets[1].block.y)) < 10) {
                //}

				offset = (targets[0].block.x + targets[1].block.x)/2.0 - 160.0;
			}
		}
	private:
		double offset;
	} strafePIDSource;

	// Deadband filter
	float deadBand(float stickValue) {
		if(stickValue <= .15 && stickValue >= -.15) {
			return 0.0;
		} else {
			return stickValue;
		}
	}

	void autoDrivingForward() {
		autoTimer++;

		if (autoTimer < autoDriveFowardTime) {
			turnController->SetSetpoint(0.0);
			turnController->Enable();
			drive->MecanumDrive_Cartesian(0.0, -kAutoSpeed, -turnPIDOutput.correction, ahrs->GetAngle());
		} else {
            autoTimer = 0;
            autoState = kTurningToGear;
			drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
        }
    }

	void autoTurningToGear() {
		autoTimer++;

		if (autoTimer < kTurningToGearTime) {
			turnController->SetSetpoint(autoTurnToGearAngle);
			turnController->Enable();
			drive->MecanumDrive_Cartesian(0.0, 0.0, -turnPIDOutput.correction * 0.5, ahrs->GetAngle());
		} else {
            autoTimer = 0;
            autoState = kDrivingToGear;
			drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
        }
    }

    void autoDrivingToGear() {
		autoTimer++;

		if (autoTimer < kDriveToGearMaxTime)
		{
			turnController->SetSetpoint(0.0);
			turnController->Enable();
			strafeController->SetSetpoint(0.0);
			strafeController->Enable();

            // Get targets from Pixy and calculate the offset from center
  		    int count = m_pixy->getBlocksForSignature(m_signature, 2, m_targets);
  		    strafePIDSource.calcTargetOffset(m_targets, count);

			drive->MecanumDrive_Cartesian(strafePIDOutput.correction, -kAutoSpeed, -turnPIDOutput.correction, 0.0);

            double curr_world_linear_accel_x = ahrs->GetWorldLinearAccelX();
            double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
            last_world_linear_accel_x = curr_world_linear_accel_x;
            double curr_world_linear_accel_y = ahrs->GetWorldLinearAccelY();
            double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
            last_world_linear_accel_y = curr_world_linear_accel_y;

            // Hit the wall!
            if ((abs(currentJerkX) > kCollisionThresholdDelta) || (abs(currentJerkY) > kCollisionThresholdDelta)) {
                autoTimer = 0;
                autoState = kPlacingGear;
			    drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
            }

		} else {
            autoTimer = 0;
            autoState = kDoNothing;
			drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
        }
    }

	void autoPlacingGear() {
		autoTimer++;
		if (autoTimer < kPlaceGearTime)
		{
			gearServoRight -> Set(kServoRightOpen);
			gearServoLeft -> Set(kServoLeftOpen);
		} else {
			autoTimer = 0;
			autoState = kDrivingBackward;
		}
	}

	void autoDrivingBackward() {
		autoTimer++;
		if (autoTimer < kDriveBackwardTime)
		{
			turnController->SetSetpoint(0.0);
			turnController->Enable();
			drive->MecanumDrive_Cartesian(0.0, kAutoSpeed, -turnPIDOutput.correction, 0.0);
		} else {
			autoTimer = 0;
			autoState = kDoNothing;
		}
	}

	void autoDoNothing() {
		drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, ahrs->GetAngle()); //stop
	}

	void RobotInit()
	{
		gearServoRight = new Servo(5); //right servo on practice bot
		gearServoLeft = new Servo(3); //left is 3 on practice bot
		climber = new TalonSRX(7);
		stick        = new Joystick(0);
		lFrontMotor  = new TalonSRX(0); // 9
		lBackMotor	 = new TalonSRX(8); // 1
		rFrontMotor  = new TalonSRX(1); // 8
		rBackMotor	 = new TalonSRX(9); // 0

		rFrontMotor->SetInverted(true);
		rBackMotor->SetInverted(true);

		try {
			ahrs = new AHRS(SPI::Port::kMXP);
		} catch (std::exception& ex ) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}

		drive = new RobotDrive(lFrontMotor, lBackMotor, rFrontMotor, rBackMotor);
		drive->SetSafetyEnabled(false);

		turnController = new PIDController(kP, kI, kD, kF, ahrs, &turnPIDOutput);
		turnController->SetInputRange(-180.0, 180.0);
		turnController->SetOutputRange(-1.0, 1.0);
		turnController->SetAbsoluteTolerance(kToleranceDegrees);
		turnController->SetContinuous(true);

		strafeController = new PIDController(ksP, ksI, ksD, ksF, &strafePIDSource, &strafePIDOutput);
		strafeController->SetInputRange(-100.0, 100.0);
		strafeController->SetOutputRange(-0.1, 0.1);
		strafeController->SetAbsoluteTolerance(kToleranceStrafe);
		strafeController->SetContinuous(true);

		// Create the Pixy instance and start streaming the frames
		m_pixy = new PixyTracker();
		m_pixy->startVideo();
	}

	void AutonomousInit() {
		m_pixy->setTiltandBrightness(kTrackingBrightness, 0);
		autoState = kDrivingForward; // Initial state
		ahrs->ZeroYaw();             // Initialize to zero

		autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected.find("r") != std::string::npos) {
			std::cout << "right\n";
            autoDriveFowardTime = 50;
			autoTurnToGearAngle = -60.0;
		} else if (autoSelected.find("l") != std::string::npos){
			std::cout << "left\n";
            autoDriveFowardTime = 50;
			autoTurnToGearAngle = 60.0;
		} else {
			std::cout << "center\n";
            autoDriveFowardTime = 0;
			autoTurnToGearAngle = 0.0;
		}

		autoTimer = 0;
	}


	void DisabledInit() {
		drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
	}

	void AutonomousPeriodic() {
		switch (autoState) {
		case kDrivingForward:
			autoDrivingForward();
			break;
		case kPlacingGear:
			autoPlacingGear();
			break;
		case kDrivingBackward:
			autoDrivingBackward();
			break;
		case kTurningToGear:
			autoTurningToGear();
			break;
		case kDrivingToGear:
			autoDrivingToGear();
			break;
		default:
		case kDoNothing:
			autoDoNothing();
			break;
		}
	}

	void TeleopInit() {
		m_pixy->setTiltandBrightness(kNormalBrightness, 1);
		m_pixy->clearTargets();
	}

	void TeleopPeriodic() {
		if (stick -> GetRawButton(6)) {
			gearServoRight -> Set(kServoRightOpen);
			gearServoLeft -> Set(kServoLeftOpen);

		} else {
			gearServoRight -> Set(kServoRightClosed);
			gearServoLeft -> Set(kServoLeftClosed);
		}

		if (stick->GetRawAxis(2)){
			climber -> Set(1.0);
		} else{
			climber -> Set(0.0);
		}

		// Check for drive-mode toggle
		/*
		if (stick->GetRawButton(1)) {
			if (!wasButtonPushed) {
				isRobotCentric = !isRobotCentric;

				if (isRobotCentric) {
					driveAngle = ahrs->GetAngle();
					double rotations = floor(abs(driveAngle)/360.0);
					if (driveAngle < 0.0) {
						driveAngle+= rotations*360.0;
					} else {
						driveAngle-= rotations*360.0;
					}
					if (driveAngle > 180.0) {
						driveAngle-=360.0;
					} else if (driveAngle < -180.0) {
						driveAngle+=360.0;
					}

				std::cout << "set drive angle\n";
				}
			}

			wasButtonPushed = true;
		} else {
			wasButtonPushed = false;
		}*/

		if (stick->GetRawButton(1) || stick->GetRawButton(2)) {
			if (!isRobotCentric) {

				isRobotCentric = true;

				driveAngle = ahrs->GetAngle();
				double rotations = floor(abs(driveAngle)/360.0);
				if (driveAngle < 0.0) {
					driveAngle+= rotations*360.0;
				} else {
					driveAngle-= rotations*360.0;
				}
				if (driveAngle > 180.0) {
					driveAngle-=360.0;
				} else if (driveAngle < -180.0) {
					driveAngle+=360.0;
				}
			}
		} else {
			isRobotCentric = false;
		}

		if (isRobotCentric) {
			std::cout << "Robot Centric: " << ahrs->GetAngle() << " " << driveAngle << " " << turnPIDOutput.correction << std::endl;
			turnController->SetSetpoint(driveAngle);
			turnController->Enable();

			if (stick->GetRawButton(2)) {
				drive->MecanumDrive_Cartesian(
								deadBand(0.0),
								deadBand(stick->GetRawAxis(1)),
								-turnPIDOutput.correction,
								0.0);
								//ahrs->GetAngle());
			} else {
			drive->MecanumDrive_Cartesian(
					deadBand(stick->GetRawAxis(0)),
					deadBand(stick->GetRawAxis(1)),
					-turnPIDOutput.correction,
					0.0);
					//ahrs->GetAngle());
			}
		} else {
			turnController->Disable();

		drive->MecanumDrive_Cartesian(
				deadBand(stick->GetRawAxis(0)),
				deadBand(stick->GetRawAxis(1)),
				kSpinRateLimiter * (-1)*deadBand(stick->GetRawAxis(4)),
				ahrs->GetAngle());
		}
		// DEBUG
		//std::cout << std::setprecision(3) << std::fixed;
		//std::cout << "Axis 0: " << stick->GetRawAxis(0) << "  Axis 1: " << stick->GetRawAxis(1) << "  Axis 4: " << stick->GetRawAxis(4) << std::endl;

		//std::cout << "Motors = LF: " << lFrontMotor->Get() << "  RF: " << rFrontMotor->Get() << "  LR: " <<
		//		lBackMotor->Get() << "  RR: " << rBackMotor->Get() << std::endl;

		//std::cout << "Angle: " << ahrs->GetAngle() << "  Rate: " << ahrs->GetRate() << std::endl;
		//std::cout << "Angle: " << ahrs->GetAngle()
		//		  << "  Yaw: " << ahrs->GetYaw()
		//		  << "  Pitch: " << ahrs->GetPitch()
		//		  << "  Roll: " << ahrs->GetRoll() << std::endl;
	}
};

START_ROBOT_CLASS(Robot);
