// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.IDConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
	static double targetZOffset = -0.0254;

	/** Creates a new ShooterSubsystem. */
	private static TalonFX topMotor, bottomMotor, feedMotor, elevatorMotor, angleMotor;

	private static CANcoder angleCanCoder;

	private static DigitalInput shooterSensor;

	private static double requestedElevatorPos = 0;
	private static double requestedArmPos = 0;
	static StatusSignal<Double> velocitySignalTop, velocitySignalBottom, velocitySignalFeed, positionSignalElevator,
			positionSignalAngle;

	static VelocityVoltage velocityVoltageControlTop = new VelocityVoltage(0).withEnableFOC(true),
			velocityVoltageControlBottom = new VelocityVoltage(0).withEnableFOC(true),
			velocityVoltageControlFeed = new VelocityVoltage(0).withEnableFOC(true);
	static VoltageOut voltageOutControlTop = new VoltageOut(0).withEnableFOC(true),
			voltageOutControlBottom = new VoltageOut(0).withEnableFOC(true),
			voltageOutControlFeed = new VoltageOut(0).withEnableFOC(true);

	public ShooterSubsystem() {

		angleCanCoder = new CANcoder(IDConstants.angleCanCoderID, "rio");
		topMotor = new TalonFX(IDConstants.topMotor, "rio");
		topMotor.getConfigurator().apply(new TalonFXConfiguration());
		topMotor.setInverted(false);
		bottomMotor = new TalonFX(IDConstants.bottomMotor, "rio");
		bottomMotor.getConfigurator().apply(new TalonFXConfiguration());
		bottomMotor.setInverted(false);
		feedMotor = new TalonFX(IDConstants.feedMotor, "rio");
		feedMotor.getConfigurator().apply(new TalonFXConfiguration());
		feedMotor.setInverted(true);
		feedMotor.setNeutralMode(NeutralModeValue.Brake);
		elevatorMotor = new TalonFX(IDConstants.elevatorMotorID, "rio");
		elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
		angleMotor = new TalonFX(IDConstants.angleMotorID, "rio");
		angleMotor.setInverted(true);
		angleMotor.setNeutralMode(NeutralModeValue.Brake);

		reloadFromConfig();

		elevatorMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
				.withForwardSoftLimitThreshold(ShooterConstants.elevatorSoftMax).withReverseSoftLimitEnable(true)
				.withReverseSoftLimitThreshold(ShooterConstants.elevatorSoftMin));

		elevatorMotor.getConfigurator().apply(new HardwareLimitSwitchConfigs().withReverseLimitEnable(true)
				.withReverseLimitAutosetPositionEnable(true).withReverseLimitAutosetPositionValue(0));

		angleMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
				.withForwardSoftLimitThreshold(ShooterConstants.angleShooterSoftMax).withReverseSoftLimitEnable(true)
				.withReverseSoftLimitThreshold(ShooterConstants.angleShooterSoftMin));

		angleMotor.getConfigurator()
				.apply(new FeedbackConfigs().withFeedbackRemoteSensorID(IDConstants.angleCanCoderID)
						.withRotorToSensorRatio(ShooterConstants.angleMotorToEncoder).withSensorToMechanismRatio(1)
						.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));

		shooterSensor = new DigitalInput(IDConstants.shooterSensorChannel);
	}

	public static void reloadFromConfig() {

		angleCanCoder.getConfigurator()
				.apply(new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
						.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
						.withMagnetOffset(ShooterConstants.shooterAngleOffset));
		elevatorMotor.getConfigurator()
				.apply(new SlotConfigs().withKP(ShooterConstants.elevatorMotorP).withKI(ShooterConstants.elevatorMotorI)
						.withKD(ShooterConstants.elevatorMotorD).withKV(ShooterConstants.elevatorMotorV)
						.withKG(ShooterConstants.elevatorMotorG).withGravityType(GravityTypeValue.Elevator_Static));
		elevatorMotor.getConfigurator()
				.apply(new MotionMagicConfigs().withMotionMagicAcceleration(256).withMotionMagicCruiseVelocity(256));

		angleMotor.getConfigurator()
				.apply(new MotionMagicConfigs().withMotionMagicAcceleration(1.0).withMotionMagicCruiseVelocity(1.0));

		angleMotor.getConfigurator()
				.apply(new SlotConfigs().withKP(ShooterConstants.angleShooterP).withKI(ShooterConstants.angleShooterI)
						.withKD(ShooterConstants.angleShooterD).withKV(ShooterConstants.angleShooterV)
						.withKG(ShooterConstants.angleShooterG).withGravityType(GravityTypeValue.Arm_Cosine));

		topMotor.getConfigurator()
				.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(45).withSupplyCurrentLimitEnable(true));

		bottomMotor.getConfigurator()
				.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(45).withSupplyCurrentLimitEnable(true));

		feedMotor.getConfigurator()
				.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(45).withSupplyCurrentLimitEnable(true));

		topMotor.getConfigurator()
				.apply(new SlotConfigs().withKP(ShooterConstants.shootP).withKV(ShooterConstants.shootV));
		bottomMotor.getConfigurator()
				.apply(new SlotConfigs().withKP(ShooterConstants.shootP).withKV(ShooterConstants.shootV));

		feedMotor.getConfigurator()
				.apply(new SlotConfigs().withKP(ShooterConstants.feedP).withKV(ShooterConstants.feedV));

		velocitySignalBottom = bottomMotor.getVelocity();
		velocitySignalTop = topMotor.getVelocity();
		velocitySignalFeed = feedMotor.getVelocity();

		positionSignalElevator = elevatorMotor.getPosition();
		positionSignalAngle = angleCanCoder.getAbsolutePosition();
	}

	public static void setArmBreakMode(boolean enabled) {
		if (enabled) {
			angleMotor.setNeutralMode(NeutralModeValue.Brake);
		} else {
			angleMotor.setNeutralMode(NeutralModeValue.Coast);
		}
	}

	public static void setElevatorBreakMode(boolean enabled) {
		if (enabled) {
			elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
		} else {
			elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
		}
	}

	public static void setTopMotorSpeed(double rps) {
		topMotor.setControl(velocityVoltageControlTop.withVelocity(rps));
	}

	public static void setTopMotorVoltage(double voltage) {
		topMotor.setControl(voltageOutControlTop.withOutput(voltage));
	}

	public static void setBottomMotorSpeed(double rps) {
		bottomMotor.setControl(velocityVoltageControlBottom.withVelocity(rps));
	}

	public static void setBottomMotorVoltage(double voltage) {
		bottomMotor.setControl(voltageOutControlBottom.withOutput(voltage));
	}

	public static void setFeedMotorSpeed(double rps) {
		feedMotor.setControl(velocityVoltageControlFeed.withVelocity(rps));
	}

	public static void setFeedMotorVoltage(double voltage) {
		feedMotor.setControl(voltageOutControlFeed.withOutput(voltage));
	}

	public static double getFeedMotorSpeed() {
		velocitySignalFeed.refresh();
		return velocitySignalFeed.getValue();
	}

	public static void setShooterAngle(double angle) {
		requestedArmPos = angle;
	}

	public static boolean getShooterSensor() {
		if (IntakeConstants.invertSensors == 1)
			return shooterSensor.get();
		return !shooterSensor.get();
	}

	public static double getTopMotorSpeed() {
		velocitySignalTop.refresh();
		return velocitySignalTop.getValue();
	}

	public static double getBottomMotorSpeed() {
		velocitySignalBottom.refresh();
		return velocitySignalBottom.getValue();
	}

	public static void setElevatorPosition(double request) {
		requestedElevatorPos = request;
	}

	public static double getElevatorPosition() {
		positionSignalElevator.refresh();
		return positionSignalElevator.getValue() * ShooterConstants.elevatorMotorToInches;
	}

	public static void initializeArmAngle() {
		requestedArmPos = getShooterAngle();
	}

	public static void initializeElevatorPosition() {
		requestedElevatorPos = getElevatorPosition();
	}

	public static double getEstimatedShooterAngle() {
		double distanceToTarget = 0;
		distanceToTarget = RobotContainer.driveSubsystem.getPose2d().getTranslation()
				.getDistance(DriveSubsystem.getOffsetTarget());
		SmartDashboard.putNumber("/Drivetrain/DistanceToTarget", distanceToTarget);

		// double angleToTarget = (Math.atan2(0.64135 - 2.0452 + targetZOffset,
		// distanceToTarget - 0.3048 - 0.2286)
		// + Math.toRadians(0 - Math.min(Math.max(0, distanceToTarget - 0.3048 - 1) * 1,
		// 10))) * 180 / Math.PI;

		double angleToTarget = -101 + 42.6667 * distanceToTarget - 8.25 * Math.pow(distanceToTarget, 2)
				+ 0.5833 * Math.pow(distanceToTarget, 3);
		angleToTarget = Math.max(angleToTarget, -55);
		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
			angleToTarget += 0;
		}

		return angleToTarget;

	}

	public static void log() {
		SmartDashboard.putBoolean("/Shooter/ShooterSensor", getShooterSensor());
		SmartDashboard.putNumber("/Shooter/FeederRPS", getFeedMotorSpeed());
		SmartDashboard.putNumber("/Shooter/TopShooterRPS", getTopMotorSpeed());
		SmartDashboard.putNumber("/Shooter/BottomShooterRPS", getBottomMotorSpeed());
		SmartDashboard.putNumber("/Shooter/ShooterAngle", getShooterAngle());
		SmartDashboard.putNumber("/Shooter/TargetShooterAngle", requestedArmPos);
		SmartDashboard.putNumber("/Shooter/ElevatorPosition", getElevatorPosition());
		SmartDashboard.putNumber("/Shooter/TargetElevatorPosition", requestedElevatorPos);
		SmartDashboard.putNumber("/Drivetrain/EstimatedAngle", getEstimatedShooterAngle());
	}

	public static double degreesToShooterRotations(double degrees) {
		return Units.degreesToRotations(degrees - ShooterConstants.restShooterAngle)
				+ ShooterConstants.shooterRestingRotations;
	}

	public static double getShooterAngle() {
		positionSignalAngle.refresh();
		return Units.rotationsToDegrees(positionSignalAngle.getValue() - ShooterConstants.shooterRestingRotations)
				+ ShooterConstants.restShooterAngle;
	}

	MotionMagicVoltage elevatorControl = new MotionMagicVoltage(0);
	MotionMagicVoltage angleControl = new MotionMagicVoltage(0);
	@Override
	public void periodic() {
		elevatorMotor.setControl(
				elevatorControl.withPosition((requestedElevatorPos / ShooterConstants.elevatorMotorToInches)));

		angleMotor.setControl(angleControl.withPosition(degreesToShooterRotations(requestedArmPos)));
	}
}
