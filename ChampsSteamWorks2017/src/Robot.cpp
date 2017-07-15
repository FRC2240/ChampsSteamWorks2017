#include <iostream>
#include <iomanip>
#include <chrono>
#include "WPILib.h"
#include "PixyTracker.hpp"
#include "AHRS.h"
#include "kalman.h"
#include <CANTalon.h>
#include <SmartDashboard/SmartDashboard.h>
#include "log.h"

const double PI = 3.1415926;
const double DEG_TO_RAD = PI/180.0;
const double PIXY_FIELD_OF_VIEW = 75.0;
const double DISTANCE_PIXY_TO_ROBOT_CENTER = 9.75;
const double TARGET_WIDTH_INCHES = 8.25;
const double PIXY_IMAGE_WIDTH_PIXELS = 320.0;
const double TARGET_HEIGHT_INCHES = 13.25;
const double PIXY_HEIGHT_INCHES = 30.5;

class Robot: public IterativeRobot {
private:

	Preferences *prefs;
	
	Servo *gearServoRight;
	Servo *gearServoLeft;
	TalonSRX *feederMotor;
	CANTalon *shooter;
	double speed;
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
	bool lastButton4 = false;
	bool lastButton2 = false;
	Kalman *filter;

	double maxX = 0.0;
	double maxY = 0.0;
	
	// Servo constants
	const double kServoRightOpen   = 0.3;
	const double kServoRightClosed = 0.0;
	const double kServoLeftOpen    = 0.0;
	const double kServoLeftClosed  = 0.3;


	// Tunable parameters for target detection
	const int kTrackingBrightness = 22;
	const int kNormalBrightness   = 128;
	const int kGearTargetWidth    = 20;
	const int kGearTargetHeight   = 50;
	const int kGearTargetArea = (kGearTargetWidth * kGearTargetHeight);

	// Tunable parameters for driving
	const double kSpinRateLimiter = 0.5;

	double autoTurnToGearAngle = 0.0;

	// Tunable parameters for autonomous
	const double kCollisionThresholdDelta = 1.0;
	const int kTurningToGearTime          = 70;
	const int kPlaceGearTime              = 50;
	const int kDriveBackwardTime          = 200;

	// Tunable parameters
	int autoDriveFowardTime;
	double kAutoSpeed;
	double kDrift;
	double kSpin;	
	double kStrafeOutputRange;
	double kTurnOutputRange;
	int kDriveToGearMaxTime;
	double kBoilerTargetWidth;
	int kBoilerTargetHeight;
	double kBoilerTargetOffset;

	// Tunable parameters for the Strafe PID Controller
	double ksP = 0.03;
	double ksI = 0.0;
	double ksD = 0.12;
	double ksF = 0.0;

	// Tunable parameters for the Turn PID Controller
	double kP = 0.01;
	double kI = 0.00;
	double kD = 0.03;
	double kF = 0.00;
	
	double kKalmanProcessNoise;
	double kKalmanSensorNoise;

	/* This tuning parameter indicates how close to "on target" the    */
	/* PID Controller will attempt to get.                             */
	const double kToleranceDegrees = 0.25;
	const double kToleranceStrafe = 0.01;

	enum AutoState {
		kDoNothing,
		kDrivingForward,
		kPlacingGear,
		kTurningToGear,
		kDrivingToGear,
		kAutoReleaseGear,
		kDrivingToBoiler
	} autoState = kDoNothing;

	PixyTracker *m_pixy;
	int m_signature = 1;
	PixyTracker::Target m_targets[2];

	RobotDrive *drive;
	TalonSRX   *lFrontMotor;
	TalonSRX   *lBackMotor;
	TalonSRX   *rFrontMotor;
	TalonSRX   *rBackMotor;
	//CANTalon   *flooper;
	Joystick   *stick;
	AHRS       *ahrs; // navX MXP

	PIDController *turnController   = NULL;    // PID Controller
	PIDController *strafeController = NULL;    // PID Controller

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
		StrafePIDSource() : scale(0.0), pixelOffset(0), offset(0.0) {}
		double PIDGet() {
			return offset;
		}
		void calcTargetOffset(PixyTracker::Target* targets, int count, double theta) {
			offset = 0.0;
			if (count >= 2) {
				double target_width_pixels = abs(targets[0].block.x - targets[1].block.x);
				double target_width_angle = 0.5*(PIXY_FIELD_OF_VIEW/PIXY_IMAGE_WIDTH_PIXELS)*target_width_pixels;
				double distance_pixy_to_target = (TARGET_WIDTH_INCHES/2.0)/tan(target_width_angle*DEG_TO_RAD);
				double height_target_to_pixy = PIXY_HEIGHT_INCHES - TARGET_HEIGHT_INCHES;
				double sc = sqrt(distance_pixy_to_target*distance_pixy_to_target - height_target_to_pixy*height_target_to_pixy);
				double scale = TARGET_WIDTH_INCHES/target_width_pixels;
				double pixelOffset = (targets[0].block.x + targets[1].block.x)/2.0 - PIXY_IMAGE_WIDTH_PIXELS/2.0;

				double target_offset_inches = scale*pixelOffset;
				double thetab;
				if (target_offset_inches < 0.0)
					thetab = DEG_TO_RAD*(90.0+theta);
				else
					thetab = DEG_TO_RAD*(90.0-theta);
				
				double alpha = asin(target_offset_inches/sc*sin(thetab));
				double k = PI-alpha-thetab;
				double z = sin(k)/sin(thetab)*sc;
				double tlen = z + 9.75;
				double offz = sin(DEG_TO_RAD*theta)*tlen;
				offset = target_offset_inches + offz;
				
				FILE_LOG(logDEBUG) << "[calcTargetOffset] target width (deg): " << target_width_angle
						<< ", distance: " << distance_pixy_to_target << ", pixel offset: " << pixelOffset
						<< ", target offset: " << target_offset_inches << ", act off: " << offset;
			}
		}
	private:	
		double scale;
		double pixelOffset;
		double offset;
	} strafePIDSource;

	// Read data from the Preferences Panel
	void getPreferences()
	{
		kDriveToGearMaxTime = prefs->GetInt("kDriveToGearMaxTime", 200);
		ksP = prefs->GetDouble("ksP", 0.02);
		ksI = prefs->GetDouble("ksI", 0.0);
		ksD = prefs->GetDouble("ksD", 0.2);
		ksF = prefs->GetDouble("ksF", 0.0);

		kP = prefs->GetDouble("kP", 0.02);
		kI = prefs->GetDouble("kI", 0.0);
		kD = prefs->GetDouble("kD", 0.2);
		kF = prefs->GetDouble("kF", 0.0);

		kAutoSpeed = prefs->GetDouble("kAutoSpeed", 0.42);
		kDrift = prefs->GetDouble("kDrift", 0.05);
		kStrafeOutputRange = prefs->GetDouble("kStrafeOutputRange", 0.3);
		kTurnOutputRange = prefs->GetDouble("kTurnOutputRange", 0.2);

		kBoilerTargetWidth = prefs->GetDouble("kBoilerTargetWidth", 70.0);
		kBoilerTargetHeight = prefs->GetDouble("kBoilerTargetHeight", 20.0);
		kBoilerTargetOffset = prefs->GetDouble("kBoilerTargetOffset", 9.0);
		kKalmanProcessNoise = prefs->GetDouble("kKalmanProcessNoise", 0.05);
		kKalmanSensorNoise = prefs->GetDouble("kKalmanSensorNoise", 8.0);
	}

	// Deadband filter
	float deadBand(float stickValue) {
		if(stickValue <= .15 && stickValue >= -.15) {
			return 0.0;
		} else {
			return stickValue;
		}
	}

	// Drive forward for pre-defined time
	void autoDrivingForward() {
		autoTimer++;

		if (autoTimer < autoDriveFowardTime) {
			turnController->SetSetpoint(0.0);
			turnController->Enable();
			drive->MecanumDrive_Cartesian(kDrift, -kAutoSpeed, -turnPIDOutput.correction, 0.0);
		} else {
			autoTimer = 0;
			autoState = kTurningToGear;
			drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
				
			if (autoSelected.find("N") != std::string::npos){
				autoState = kDoNothing;
			}
		}
	}

	// Turn toward the gear using the pre-defined angle
	void autoTurningToGear() {
		autoTimer++;

		if (autoTimer < kTurningToGearTime) {
			//std::cout << "  ANGLE: " << ahrs -> GetAngle() << std::endl;
			turnController->SetSetpoint(autoTurnToGearAngle);
			turnController->Enable();
			drive->MecanumDrive_Cartesian(0.0, 0.0, -turnPIDOutput.correction, ahrs->GetAngle());
		} else {
			//std::cout << "  ANGLE: " << ahrs -> GetAngle() << std::endl;
			autoTimer = 0;
			autoState = kDrivingToGear;
			driveAngle = autoTurnToGearAngle; //ahrs->GetAngle();
			drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
		}
	}

	// Drive forward to gear using Pixy and gyro for guidance
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
			
			for (int i=0; i<count; i++) {
				FILE_LOG(logDEBUG) << "[Gear Target " << i << "] x: " << m_targets[i].block.x << ", y: " << m_targets[i].block.y
								   << ", w: " << m_targets[i].block.width << ", h: " << m_targets[i].block.height;
			}
			
			strafePIDSource.calcTargetOffset(m_targets, count, ahrs->GetAngle()-driveAngle);	

			drive->MecanumDrive_Cartesian(kDrift-strafePIDOutput.correction, -kAutoSpeed, kSpin-turnPIDOutput.correction, 0.0);
			
			FILE_LOG(logDEBUG) << "[autoDrivingToGear] timer: " << autoTimer << ", strafe correction: " << strafePIDOutput.correction
					           << ", turn correction: " << turnPIDOutput.correction
							   << ", drive angle: " << driveAngle << ", gyro angle: " << ahrs->GetAngle();

			double curr_world_linear_accel_x = ahrs->GetWorldLinearAccelX();
			double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
			last_world_linear_accel_x = curr_world_linear_accel_x;
			double curr_world_linear_accel_y = ahrs->GetWorldLinearAccelY();
			double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
			last_world_linear_accel_y = curr_world_linear_accel_y;
			
			currentJerkX = fabs(currentJerkX);
			currentJerkY = fabs(currentJerkY);
			if (currentJerkX > maxX) {
				maxX = currentJerkX;
			}
			if (currentJerkY > maxY) {
				maxY = currentJerkY;
			}

			// Hit the wall!
			if (autoTimer > 40) {
				if ((currentJerkX > kCollisionThresholdDelta) || (currentJerkY > kCollisionThresholdDelta)) {
					autoTimer = 0;
					autoState = kPlacingGear;
					drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
				}
			}

		} else {
			FILE_LOG(logDEBUG) << "[autoDrivingToGear] END";
			autoTimer = 0;
			autoState = kPlacingGear;
			drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
		}
	}

	// Open the servos
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

	void autoDoNothing() {
		drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, ahrs->GetAngle()); //stop
	}
	
	// Release the gear
	void autoReleaseGear() {
		autoTimer++;
		//std::cout << "AUTO RELEASE GEAR: " << flooper->GetPosition() << std::endl;

		if (autoTimer == 1) {
			////double targetPositionRotations = 25.0;
			//flooper->SetControlMode(CANSpeedController::kPosition);
			//flooper->Set(18.0);
			////flooper->Set(targetPositionRotations);
		} else if (autoTimer < 75) {
			turnController->SetSetpoint(driveAngle);
			turnController->Enable();
			drive->MecanumDrive_Cartesian(-kDrift, 0.5, -turnPIDOutput.correction, 0.0);
		} else if (autoTimer == 75) {
			drive->MecanumDrive_Cartesian(0.0, 0.0, -turnPIDOutput.correction, 0.0);
			//flooper->SetControlMode(CANSpeedController::kPosition);
			//flooper->Set(0.0);
		} else if (autoTimer > 100) {
			//flooper->SetControlMode(CANSpeedController::kPercentVbus);
			//flooper->Set(0.0);
			autoTimer = 0;
			autoState = kDoNothing;
		}			
	}
	
	void autoDrivingToBoiler() {
		autoTimer++;
		turnController->Enable();

		int count = m_pixy->getBlocksForSignature(m_signature, 1, m_targets);
		driveAngle = unwrapGyroAngle(ahrs->GetAngle());
		double pixelOffset = (count) ? m_targets[0].block.x - PIXY_IMAGE_WIDTH_PIXELS/2.0: 0.0;
		double angleOffset = pixelOffset * (PIXY_FIELD_OF_VIEW/PIXY_IMAGE_WIDTH_PIXELS);

		FILE_LOG(logDEBUG) << "gyro angle: " << driveAngle << ", offset: " << angleOffset
				           << ", setpoint: " << turnController->GetSetpoint();

		FILE_LOG(logDEBUG) << "autoDrivingToBoiler: width: " << m_targets[0].block.width
					       << ", height: " << m_targets[0].block.height << ", center: " << m_targets[0].block.x;

		if (autoTimer < 100) {
			// just turning
			turnController->SetSetpoint(driveAngle + angleOffset + kBoilerTargetOffset);
			drive->MecanumDrive_Cartesian(0.0, 0.0, -turnPIDOutput.correction, 0.0);
			if (angleOffset < 2.0) {
				// start driving
				autoTimer = 100;
			}
		} else if (autoTimer < 300) {
			// driving forward
			turnController->SetSetpoint(driveAngle + angleOffset + kBoilerTargetOffset);
			drive->MecanumDrive_Cartesian(kDrift, -kAutoSpeed, -turnPIDOutput.correction, 0.0);

			// At correct distance?
			double filtered = filter->getFilteredValue(double(m_targets[0].block.width));
			FILE_LOG(logDEBUG) << "time: " << autoTimer << ", width: " << m_targets[0].block.width << " " << filtered;

			if (filtered > kBoilerTargetWidth) {
				drive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
				autoTimer = 300;
			}
		} else if (autoTimer < 400) {
			// settle down
			turnController->SetSetpoint(driveAngle + angleOffset + kBoilerTargetOffset);
			drive->MecanumDrive_Cartesian(0.0, 0.0, -turnPIDOutput.correction, 0.0);
		} else {
			autoState = kDoNothing;
			autoTimer = 0;
		}
	}

	// Convert gyro to range of -180 to +180
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

	// Reset the 'strafe' and 'turn' PID controllers
	void resetPIDControllers() {
		delete strafeController;
		delete turnController;
		delete filter;
		
		strafeController = new PIDController(ksP, ksI, ksD, ksF, &strafePIDSource, &strafePIDOutput);
		strafeController->SetInputRange(-100.0, 100.0);
		strafeController->SetOutputRange(-kStrafeOutputRange, kStrafeOutputRange);
		strafeController->SetAbsoluteTolerance(kToleranceStrafe);
		strafeController->SetContinuous(true);

		turnController = new PIDController(kP, kI, kD, kF, ahrs, &turnPIDOutput);
		turnController->SetInputRange(-180.0, 180.0);
		turnController->SetOutputRange(-kTurnOutputRange, kTurnOutputRange);
		turnController->SetAbsoluteTolerance(kToleranceDegrees);
		turnController->SetContinuous(true);
		
		filter = new Kalman(kKalmanProcessNoise, kKalmanSensorNoise, 1000.0, 20.0);
	}

	void RobotInit()
	{
		prefs = Preferences::GetInstance();
		
		gearServoRight = new Servo(5);
		gearServoLeft = new Servo(3);
		climber = new TalonSRX(7);
		stick        = new Joystick(0);
		lFrontMotor  = new TalonSRX(0); // 9
		lBackMotor	 = new TalonSRX(8); // 1
		rFrontMotor  = new TalonSRX(1); // 8
		rBackMotor	 = new TalonSRX(9); // 0
		//flooper      = new CANTalon(0);
		
		feederMotor = new TalonSRX(6);

		rFrontMotor->SetInverted(true);
		rBackMotor->SetInverted(true);
		shooter = new CANTalon(0);
		shooter -> SetTalonControlMode(CANTalon::kSpeedMode);
		shooter -> Set(3700.0);
		shooter -> SetSensorDirection(true);
		shooter -> ConfigNominalOutputVoltage(+0.0, -0.0);
		
		shooter-> ConfigPeakOutputVoltage(+12., 0.0);
		shooter-> SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		shooter->SetF(0.024);	// 775 Pro
		shooter->SetP(0.035);	// 775 Pro
		shooter->SetI(0.00);	// 775 Pro
		shooter->SetD(0.1);		

		try {
			ahrs = new AHRS(SPI::Port::kMXP);
		} catch (std::exception& ex ) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}

		drive = new RobotDrive(lFrontMotor, lBackMotor, rFrontMotor, rBackMotor);
		drive->SetSafetyEnabled(false);
				
		//int absolutePosition = flooper->GetPulseWidthPosition() & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		/* use the low level API to set the quad encoder signal */
		//flooper->SetEncPosition(absolutePosition);

		/* choose the sensor and sensor direction */
		//flooper->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		//flooper->SetSensorDirection(true);
		////_talon->ConfigEncoderCodesPerRev(XXX), // if using FeedbackDevice.QuadEncoder
		////_talon->ConfigPotentiometerTurns(XXX), // if using FeedbackDevice.AnalogEncoder or AnalogPot

		/* set the peak and nominal outputs, 12V means full */
		//flooper->ConfigNominalOutputVoltage(+0., -0.);
		//flooper->ConfigPeakOutputVoltage(+8., -8.);
		/* set the allowable closed-loop error,
		 * Closed-Loop output will be neutral within this range.
		 * See Table in Section 17.2.1 for native units per rotation.
		 */
		//flooper->SetAllowableClosedLoopErr(0); /* always servo */
		/* set closed loop gains in slot0 */
		//flooper->SelectProfileSlot(0);
		//flooper->SetF(0.0);
		//flooper->SetP(0.1);
		//flooper->SetI(0.0);
		//flooper->SetD(0.0);
		
		// Create the Pixy instance and start streaming the frames
		m_pixy = new PixyTracker();
		m_pixy->startVideo();
	}

	void AutonomousInit() {
		maxX = 0;
		maxY = 0;
		m_pixy->setTiltandBrightness(kTrackingBrightness, 0);
		autoState = kDrivingForward; // Initial state
		ahrs->ZeroYaw();             // Initialize to zero
		
		getPreferences();

		FILE_LOG(logDEBUG) << "[AutoInit] kDriveToGearMaxTime: " << kDriveToGearMaxTime << ", ksP: " << ksP
						   << ", ksI: " << ksI << ", ksD: " << ksD << ", kAutoSpeed: " << kAutoSpeed
						   << ", kDrift: " << kDrift << ", kStrafeOutputRange: " << kStrafeOutputRange;
		FILE_LOG(logDEBUG) << "[AutoInit] kP: " << kP << ", kI: " << kI << ", kD: " << kD;
		
		resetPIDControllers();	

		autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		FILE_LOG(logDEBUG) << "Auto selected: " << autoSelected;

		if (autoSelected.find("R") != std::string::npos) {
			FILE_LOG(logDEBUG) << "AUTO RIGHT GEAR";
			autoDriveFowardTime = 140;
			autoTurnToGearAngle = -60.0;
		} else if (autoSelected.find("L") != std::string::npos){
			FILE_LOG(logDEBUG) << "AUTO LEFT GEAR";
			autoDriveFowardTime = 140;
			autoTurnToGearAngle = 60.0;
		}
		else if (autoSelected.find("N") != std::string::npos){
			FILE_LOG(logDEBUG) << "AUTO NO GEAR";
		} else {
			FILE_LOG(logDEBUG) << "AUTO RIGHT CENTER";
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
		//pixy->setTiltandBrightness(kNormalBrightness, 1);
		m_pixy->clearTargets();
		autoState = kDoNothing;
		
		getPreferences();

		FILE_LOG(logDEBUG) << "[TeleopInit] kDriveToGearMaxTime: " << kDriveToGearMaxTime << ", ksP: " << ksP
						   << ", ksI: " << ksI << ", ksD: " << ksD << ", kAutoSpeed: " << kAutoSpeed
						   << ", kDrift: " << kDrift << ", kStrafeOutputRange: " << kStrafeOutputRange;
		FILE_LOG(logDEBUG) << "[TeleopInit] kP: " << kP << ", kI: " << kI << ", kD: " << kD;

		FILE_LOG(logDEBUG) << "[TeleopInit] gyro angle: " << ahrs->GetAngle();
		
		resetPIDControllers();
	}

	void TeleopPeriodic() {
		bool button6 = stick->GetRawButton(6);
		bool button4 = stick->GetRawButton(4);
		bool button2 = stick->GetRawButton(2);
		
		if (button6) {
			gearServoRight -> Set(kServoRightOpen);
			gearServoLeft -> Set(kServoLeftOpen);
		} else if (!button6 && lastButton6){
			autoState = kAutoReleaseGear;
			autoTimer = 0;
			driveAngle = unwrapGyroAngle(ahrs->GetAngle());
		}
		lastButton6 = button6;
		
		if (autoState == kDrivingToGear) {
			if (button4 && !lastButton4) {
				autoState = kDoNothing;
				lastButton4 = button4;				
			} else {
				autoDrivingToGear();
				lastButton4 = button4;
				return;
			}
		}
		if (autoState == kPlacingGear) {
			if (button4 && !lastButton4) {
				autoState = kDoNothing;
				lastButton4 = button4;				
			} else {
				autoPlacingGear();
				lastButton4 = button4;
				return;
			}
		}
		if (autoState == kAutoReleaseGear) {
			if (button4 && !lastButton4) {
				autoState = kDoNothing;
				lastButton4 = button4;
			} else {			
				autoReleaseGear();
				lastButton4 = button4;
				return;
			}
		}
	
		if (autoState == kDrivingToBoiler) {
			if (button2 && !lastButton2) {
				autoState = kDoNothing;
				lastButton2 = button2;
			} else {			
				autoDrivingToBoiler();
				lastButton2 = button2;
				return;
			}
		}
	
		if (button4 && !lastButton4) {
			resetPIDControllers();
			driveAngle = unwrapGyroAngle(ahrs->GetAngle());
			autoState = kDrivingToGear;
			autoTimer = 0;
			lastButton4 = button4;
			return;
		}
		if (button2 && !lastButton2) {
			resetPIDControllers();
			driveAngle = unwrapGyroAngle(ahrs->GetAngle());
			autoState = kDrivingToBoiler;
			autoTimer = 0;
			lastButton2 = button2;
			return;
		}		
		lastButton4 = button4;
		lastButton2 = button2;
		
		if (!button6) {
			gearServoRight -> Set(kServoRightClosed);
			gearServoLeft -> Set(kServoLeftClosed);
		}
		
		if (stick->GetRawAxis(2)){
			climber -> Set(1.0);
		} else{
			climber -> Set(0.0);
		}
		
		if (stick -> GetRawAxis(3)) {
			++timer;
			speed = shooter -> GetSpeed();
			shooter -> SetTalonControlMode(CANTalon::kSpeedMode);
			shooter -> Set(3700.0);

			FILE_LOG(logDEBUG) << "[Launcher] speed: " << speed;

			if (timer == 50) {
				FILE_LOG(logDEBUG) << "[Feeder] START";

				timer = 0;
				feederMotor ->Set(0.7);
			}
		} else {
			timer = 0;
			feederMotor -> Set(0.0);
			shooter -> Set(0.0);
		}		

		if (stick->GetRawButton(1)) {
			if (!isRobotCentric) {
				isRobotCentric = true;
				driveAngle = unwrapGyroAngle(ahrs->GetAngle());
			}
		} else {
			isRobotCentric = false;
		}

		if (isRobotCentric) {
			FILE_LOG(logDEBUG) << "[Robot Centric] gyro angle: " << ahrs->GetAngle()
					           << ", drive angle: " << driveAngle
							   << ", PID correction: " << turnPIDOutput.correction;
			turnController->SetSetpoint(driveAngle);
			turnController->Enable();

			drive->MecanumDrive_Cartesian(
					deadBand(stick->GetRawAxis(0)),
					deadBand(stick->GetRawAxis(1)),
					-turnPIDOutput.correction,
					0.0);
		} else {
			turnController->Disable();

			drive->MecanumDrive_Cartesian(
					deadBand(stick->GetRawAxis(0)),
					deadBand(stick->GetRawAxis(1)),
					kSpinRateLimiter * (-1)*deadBand(stick->GetRawAxis(4)),
					ahrs->GetAngle());
		}
	}
	
	void TestInit() {
		ahrs->ZeroYaw();
	}
	
	void TestPeriodic() {
		double theta = ahrs->GetAngle();
		double offset = 0.0;

		const double PI = 3.1415926;
		const double DEG_TO_RAD = (PI/180.0);
		const double PIXY_FIELD_OF_VIEW = 75.0; 
		const double DISTANCE_PIXY_TO_ROBOT_CENTER = 9.75;
		const double TARGET_WIDTH_INCHES = 8.25;
		const double PIXY_IMAGE_WIDTH_PIXELS = 320.0;
		const double TARGET_HEIGHT_INCHES = 13.25;
		const double PIXY_HEIGHT_INCHES = 30.5;
					
		int count = m_pixy->getBlocksForSignature(m_signature, 2, m_targets);
		if (count == 2) {
			std::cout << m_targets[0].block.x << " " << m_targets[0].block.y << " "
					<< m_targets[0].block.width << " " << m_targets[0].block.height << "      "
					<< m_targets[1].block.x << " " << m_targets[1].block.y << " "
					<< m_targets[1].block.width << " "
							"" << m_targets[1].block.height << std::endl;
			
			double target_width_pixels = abs(m_targets[0].block.x - m_targets[1].block.x);
			double target_width_angle = 0.5*(PIXY_FIELD_OF_VIEW/PIXY_IMAGE_WIDTH_PIXELS)*target_width_pixels;
			double distance_pixy_to_target = (TARGET_WIDTH_INCHES/2.0)/tan(target_width_angle*DEG_TO_RAD);
			double height_target_to_pixy = PIXY_HEIGHT_INCHES - TARGET_HEIGHT_INCHES;
			double sc = sqrt(distance_pixy_to_target*distance_pixy_to_target - height_target_to_pixy*height_target_to_pixy);
			double scale = TARGET_WIDTH_INCHES/target_width_pixels;
			double pixelOffset = (m_targets[0].block.x + m_targets[1].block.x)/2.0 - PIXY_IMAGE_WIDTH_PIXELS/2.0;
		
			double target_offset_inches = -scale*pixelOffset;
			double thetab;
			if (target_offset_inches < 0.0)
				thetab = DEG_TO_RAD*(90.0+theta);
			else
				thetab = DEG_TO_RAD*(90.0-theta);
			
			double alpha = asin(target_offset_inches/sc*sin(thetab));
			double k = PI-alpha-thetab;
			double z = sin(k)/sin(thetab)*sc;
			double tlen = z + DISTANCE_PIXY_TO_ROBOT_CENTER;
			double offz = -sin(DEG_TO_RAD*theta)*tlen;
			offset = target_offset_inches + offz;

			FILE_LOG(logDEBUG) << "[calcTargetOffset] target width (deg): " << target_width_angle
					<< ", distance: " << distance_pixy_to_target << ", pixel offset: " << pixelOffset
					<< ", target offset: " << target_offset_inches
					<< ", offz: " << offz
					<< ", offset: " << offset;						
			FILE_LOG(logDEBUG) << "gyro angle: " << ahrs->GetAngle();				
		}
	}
};

START_ROBOT_CLASS(Robot);
