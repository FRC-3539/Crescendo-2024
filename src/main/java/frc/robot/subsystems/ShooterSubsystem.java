// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
	double targetZOffset = -0.0254;

	/** Creates a new ShooterSubsystem. */
	private TalonFX topMotor, bottomMotor, feedMotor, elevatorMotor, angleMotor;

	private CANcoder angleCanCoder;

	private DigitalInput shooterSensor;

	private double requestedElevatorPos = 0;
	private double requestedArmPos = 0;

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

	public void reloadFromConfig() {

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
	}

	public void setArmBreakMode(boolean enabled) {
		if (enabled) {
			angleMotor.setNeutralMode(NeutralModeValue.Brake);
		} else {
			angleMotor.setNeutralMode(NeutralModeValue.Coast);
		}
	}

	public void setElevatorBreakMode(boolean enabled) {
		if (enabled) {
			elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
		} else {
			elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
		}
	}

	public void setTopMotorSpeed(double rps) {
		topMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
	}

	public void setTopMotorVoltage(double voltage) {
		topMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	public void setBottomMotorSpeed(double rps) {
		bottomMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
	}

	public void setBottomMotorVoltage(double voltage) {
		bottomMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	public void setFeedMotorSpeed(double rps) {
		feedMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
	}

	public void setFeedMotorVoltage(double voltage) {
		feedMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	public double getFeedMotorSpeed() {
		return feedMotor.getVelocity().getValue();
	}

	public void setShooterAngle(double angle) {
		requestedArmPos = angle;
	}

	public boolean getShooterSensor() {
		if (IntakeConstants.invertSensors == 1)
			return shooterSensor.get();
		return !shooterSensor.get();
	}

	public double getTopMotorSpeed() {
		return topMotor.getVelocity().getValue();
	}

	public double getBottomMotorSpeed() {
		return bottomMotor.getVelocity().getValue();
	}

	public void setElevatoPosition(double request) {
		requestedElevatorPos = request;
	}

	public double getElevatorPosition() {
		return elevatorMotor.getPosition().getValue() * ShooterConstants.elevatorMotorToInches;
	}

	public void initializeArmAngle() {
		requestedArmPos = getShooterAngle();
	}

	public void initializeElevatorPosition() {
		requestedElevatorPos = getElevatorPosition();
	}

	public double getEstimatedShooterAngle() {
		double distanceToTarget = 0;
		distanceToTarget = RobotContainer.drivetrainSubsystem.getPose2d().getTranslation()
				.getDistance(RobotContainer.drivetrainSubsystem.getOffsetTarget());
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

	public void log() {
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

	public double degreesToShooterRotations(double degrees) {
		return Units.degreesToRotations(degrees - ShooterConstants.restShooterAngle)
				+ ShooterConstants.shooterRestingRotations;
	}

	public double getShooterAngle() {
		return Units.rotationsToDegrees(
				angleCanCoder.getAbsolutePosition().getValue() - ShooterConstants.shooterRestingRotations)
				+ ShooterConstants.restShooterAngle;
	}

	@Override
	public void periodic() {
		elevatorMotor
				.setControl(new MotionMagicVoltage((requestedElevatorPos / ShooterConstants.elevatorMotorToInches)));

		angleMotor.setControl(new MotionMagicVoltage(degreesToShooterRotations(requestedArmPos)));
	}
}
