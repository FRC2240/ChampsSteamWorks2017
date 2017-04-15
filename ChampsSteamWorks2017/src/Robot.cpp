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
	bool lastButton6 = false;

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
	const double kAutoSpeed               = 0.4;
	const double kCollisionThresholdDelta = 0.5;
	const int kTurningToGearTime          = 70;
	const int kDriveToGearMaxTime         = 220;
	const int kPlaceGearTime              = 50;
	const int kDriveBackwardTime          = 50;
	const double kStrafeOutputRange       = 0.2;

	// Tunable parameters for the Strafe PID Controller
	const double ksP = 0.03; // 0.02
	const double ksI = 0.0;
	const double ksD = 0.12; //0.06
	const double ksF = 0.0;

	// Tunable parameters for the Turn PID Controller
	const double kP = 0.01;
	const double kI = 0.00;
	const double kD = 0.03;
	const double kF = 0.00;

	/* This tuning parameter indicates how close to "on target" the    */
	/* PID Controller will attempt to get.                             */
	const double kToleranceDegrees = 1.0;
	const double kToleranceStrafe = 0.01;

	enum AutoState {
		kDoNothing,
		kDrivingForward,
		kPlacingGear,
		kDrivingBackward,
		kTurningToGear,
		kDrivingToGear,
		kAutoReleaseGear
	} autoState = kDoNothing;

	PixyTracker *m_pixy;
	int m_signature = 1;
	PixyTracker::Target m_targets[2];

	RobotDrive *drive;
	TalonSRX   *lFrontMotor;
	TalonSRX   *lBackMotor;
	TalonSRX   *rFrontMotor;
	TalonSRX   *rBackMotor;
	CANTalon   *flooper;
	Joystick   *stick;
	AHRS       *ahrs; // navX MXP

	PIDController *turnController;      // PID Controller
	PIDController *strafeController;    // PID Controller

	// Output from the Turn (Angle) PID Controller
	class TurnPIDOutput : public PIDOutput {
	public:
		double correction = 0.0;
		void PIDWrite(double output) {
			correction = output;
		}
	} turnPIDOutput;

	// Output from the Strafe PID Controller
	class StrafePIDOutput : public PIDOutput {
	public:
		double correction = 0.0;
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
			offset = 0.0;
			if (count >= 2) {
				scale = 8.25/(abs(targets[0].block.x - targets[1].block.x));
				pixelOffset = (targets[0].block.x + targets[1].block.x)/2.0 - 160.0;
				offset = scale*pixelOffset;
				// std::cout << "OFFSET = " << offset << std::endl;
			}
		}
	private:
		double scale;
		double pixelOffset;
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
			drive->MecanumDrive_Cartesian(0.035, -kAutoSpeed, -turnPIDOutput.correction, 0.0);
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
			driveAngle = ahrs->GetAngle();
			drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
		}
	}

	void autoDrivingToGear() {
		autoTimer++;

		if (autoTimer < kDriveToGearMaxTime)
		{
			turnController->SetSetpoint(driveAngle);
			turnController->Enable();
			strafeController->SetSetpoint(0.0);
			strafeController->Enable();

			// Get targets from Pixy and calculate the offset from center
			int count = m_pixy->getBlocksForSignature(m_signature, 2, m_targets);
			strafePIDSource.calcTargetOffset(m_targets, count);

			drive->MecanumDrive_Cartesian(/*0.025*/-strafePIDOutput.correction, -0.35, -turnPIDOutput.correction, 0.0);
			// std::cout << "CORR = " << strafePIDOutput.correction << "  ";

			double curr_world_linear_accel_x = ahrs->GetWorldLinearAccelX();
			double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
			last_world_linear_accel_x = curr_world_linear_accel_x;
			double curr_world_linear_accel_y = ahrs->GetWorldLinearAccelY();
			double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
			last_world_linear_accel_y = curr_world_linear_accel_y;

			// Hit the wall!
			std::cout << autoTimer << "  X:" << fabs(currentJerkX) << "Y:" << fabs(currentJerkY) << std::endl;
			if (autoTimer > 80) {
				if ((fabs(currentJerkX) > kCollisionThresholdDelta) || (fabs(currentJerkY) > kCollisionThresholdDelta)) {
					autoTimer = 0;
					autoState = kPlacingGear;
					drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
				}
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
			autoState = kAutoReleaseGear;
		}
	}

	void autoDrivingBackward() {
		autoTimer++;
		if (autoTimer < kDriveBackwardTime)
		{
			turnController->SetSetpoint(driveAngle);
			turnController->Enable();
			drive->MecanumDrive_Cartesian(-0.025, kAutoSpeed, -turnPIDOutput.correction, 0.0);
		} else {
			autoTimer = 0;
			autoState = kDoNothing;
		}
	}

	void autoDoNothing() {
		drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, ahrs->GetAngle()); //stop
	}
	
	void autoReleaseGear() {
		autoTimer++;
		//std::cout << "AUTO RELEASE GEAR: " << flooper->GetPosition() << std::endl;

		if (autoTimer == 5) {
			//double targetPositionRotations = 25.0;
			flooper->SetControlMode(CANSpeedController::kPosition);
			flooper->Set(18.0);
			//flooper->Set(targetPositionRotations);
		} else if (autoTimer < 40) {
			turnController->SetSetpoint(driveAngle);
			turnController->Enable();
			drive->MecanumDrive_Cartesian(-0.025, 0.5, -turnPIDOutput.correction, 0.0);
		} else if (autoTimer < 60) {
			flooper->SetControlMode(CANSpeedController::kPosition);
			flooper->Set(-18.0);
		} else {
			flooper->SetControlMode(CANSpeedController::kPercentVbus);
			flooper->Set(0.0);
			autoTimer = 0;
			autoState = kDoNothing;
		}			
	}

	double unwrapGyroAngle(double driveAngle) {
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
		return driveAngle;
	}

	void RobotInit()
	{
		gearServoRight = new Servo(5);
		gearServoLeft = new Servo(3);
		climber = new TalonSRX(7);
		stick        = new Joystick(0);
		lFrontMotor  = new TalonSRX(0); // 9
		lBackMotor	 = new TalonSRX(8); // 1
		rFrontMotor  = new TalonSRX(1); // 8
		rBackMotor	 = new TalonSRX(9); // 0
		flooper      = new CANTalon(0);
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
		strafeController->SetOutputRange(-kStrafeOutputRange, kStrafeOutputRange);
		strafeController->SetAbsoluteTolerance(kToleranceStrafe);
		strafeController->SetContinuous(true);
		
		int absolutePosition = flooper->GetPulseWidthPosition() & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		/* use the low level API to set the quad encoder signal */
		flooper->SetEncPosition(absolutePosition);

		/* choose the sensor and sensor direction */
		flooper->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		flooper->SetSensorDirection(true);
		//_talon->ConfigEncoderCodesPerRev(XXX), // if using FeedbackDevice.QuadEncoder
		//_talon->ConfigPotentiometerTurns(XXX), // if using FeedbackDevice.AnalogEncoder or AnalogPot

		/* set the peak and nominal outputs, 12V means full */
		flooper->ConfigNominalOutputVoltage(+0., -0.);
		flooper->ConfigPeakOutputVoltage(+8., -8.);
		/* set the allowable closed-loop error,
		 * Closed-Loop output will be neutral within this range.
		 * See Table in Section 17.2.1 for native units per rotation.
		 */
		flooper->SetAllowableClosedLoopErr(0); /* always servo */
		/* set closed loop gains in slot0 */
		flooper->SelectProfileSlot(0);
		flooper->SetF(0.0);
		flooper->SetP(0.1);
		flooper->SetI(0.0);
		flooper->SetD(0.0);
		//flooper->SetControlMode(CANSpeedController::kPosition);
		
		// Create the Pixy instance and start streaming the frames
		m_pixy = new PixyTracker();
		m_pixy->startVideo();
	}

	void AutonomousInit() {
		std::cout << "auto init\n";
		m_pixy->setTiltandBrightness(kTrackingBrightness, 0);
		autoState = kDrivingForward; // Initial state
		ahrs->ZeroYaw();             // Initialize to zero

		autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected.find("R") != std::string::npos) {
			std::cout << "right\n";
			autoDriveFowardTime = 150;
			autoTurnToGearAngle = -60.0;
		} else if (autoSelected.find("L") != std::string::npos){
			std::cout << "left\n";
			autoDriveFowardTime = 170;
			autoTurnToGearAngle = 60.0;
		} else if (autoSelected.find("N") != std::string::npos){
			std::cout << "none\n";
			autoState = kAutoReleaseGear;
		} else {
			std::cout << "center\n";
			driveAngle = 0.0;
			autoState = kDrivingToGear;
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
		case kAutoReleaseGear:
			autoReleaseGear();
			break;
		default:
		case kDoNothing:
			autoDoNothing();
			break;
		}
	}

	void TeleopInit() {
		std::cout << "tele init\n";
		m_pixy->setTiltandBrightness(kNormalBrightness, 1);
		m_pixy->clearTargets();
	}

	void TeleopPeriodic() {
		bool button6 = stick->GetRawButton(6);
		
		if (button6) {
			gearServoRight -> Set(kServoRightOpen);
			gearServoLeft -> Set(kServoLeftOpen);
		} else if (!button6 && lastButton6){
			std::cout << "tele start\n";
			autoState = kAutoReleaseGear;
			autoTimer = 0;
			driveAngle = unwrapGyroAngle(ahrs->GetAngle());
			//gearServoRight -> Set(kServoRightClosed);
			//gearServoLeft -> Set(kServoLeftClosed);
		}
		lastButton6 = button6;
		if (autoState == kAutoReleaseGear) {
			std::cout << "tele rel\n";
			autoReleaseGear();
			return;
		}
		if (!button6) {
			gearServoRight -> Set(kServoRightClosed);
			gearServoLeft -> Set(kServoLeftClosed);
		}
		
		if (stick->GetRawAxis(2)){
			climber -> Set(1.0);
		} else{
			climber -> Set(0.0);
		}

		if (stick -> GetRawButton(4)){
			flooper ->Set(-.2);
		} else if(stick -> GetRawButton(3)){
			flooper -> Set(.2);
		}
		else {
			flooper -> Set(0.0);
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

				driveAngle = unwrapGyroAngle(ahrs->GetAngle());
				/*double rotations = floor(abs(driveAngle)/360.0);
				if (driveAngle < 0.0) {
					driveAngle+= rotations*360.0;
				} else {
					driveAngle-= rotations*360.0;
				}
				if (driveAngle > 180.0) {
					driveAngle-=360.0;
				} else if (driveAngle < -180.0) {
					driveAngle+=360.0;
				}*/
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
			} else {
				drive->MecanumDrive_Cartesian(
						deadBand(stick->GetRawAxis(0)),
						deadBand(stick->GetRawAxis(1)),
						-turnPIDOutput.correction,
						0.0);
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

	void TestPeriodic() {
		int count = m_pixy->getBlocksForSignature(m_signature, 2, m_targets);
		if (count == 2) {
			std::cout << m_targets[0].block.x << " " << m_targets[0].block.y << " "
					<< m_targets[0].block.width << " " << m_targets[0].block.height << "      "
					<< m_targets[1].block.x << " " << m_targets[1].block.y << " "
					<< m_targets[1].block.width << " " << m_targets[1].block.height << std::endl;
			double scale = 8.25/(abs(m_targets[0].block.x-m_targets[1].block.x));
			double pixelOffset = (m_targets[0].block.x + m_targets[1].block.x)/2.0 - 160.0;
			std::cout << "OFFSET = " << scale*pixelOffset << std::endl;
		}
	}
};

START_ROBOT_CLASS(Robot);
