// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
	/** Creates a new IntakeSubsystem. */
	private static TalonFX groundMotor, kickMotor, chamberMotor;

	private static DigitalInput frontSensor, backSensor, chamberSensor;

	static StatusSignal<Double> groundMotorVelocity, kickMotorVelocity, chamberMotorVelocity;

	public IntakeSubsystem() {
		groundMotor = new TalonFX(IDConstants.groundMotorID, "rio");
		groundMotor.getConfigurator().apply(new TalonFXConfiguration());
		groundMotor.setInverted(true);
		groundMotor.setNeutralMode(NeutralModeValue.Brake);
		kickMotor = new TalonFX(IDConstants.kickMotorID, "rio");
		kickMotor.getConfigurator().apply(new TalonFXConfiguration());
		kickMotor.setInverted(true);
		kickMotor.setNeutralMode(NeutralModeValue.Coast);

		chamberMotor = new TalonFX(IDConstants.grabMotorID, "rio");

		frontSensor = new DigitalInput(IDConstants.frontSensorChannel);
		backSensor = new DigitalInput(IDConstants.backSensorChannel);
		chamberSensor = new DigitalInput(IDConstants.chamberSensorChannel);
		reloadFromConfig();

		groundMotorVelocity = groundMotor.getVelocity();
		kickMotorVelocity = kickMotor.getVelocity();
		chamberMotorVelocity = chamberMotor.getVelocity();
	}

	public static void reloadFromConfig() {
		groundMotor.getConfigurator()
				.apply(new SlotConfigs().withKP(IntakeConstants.groundP).withKV(IntakeConstants.groundV));
		chamberMotor.getConfigurator()
				.apply(new SlotConfigs().withKP(IntakeConstants.chamberP).withKV(IntakeConstants.chamberV));
		kickMotor.getConfigurator()
				.apply(new SlotConfigs().withKP(IntakeConstants.kickP).withKV(IntakeConstants.kickV));

		chamberMotor.getConfigurator()
				.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(45).withSupplyCurrentLimitEnable(true));
		groundMotor.getConfigurator()
				.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(120).withSupplyCurrentLimitEnable(false));
		kickMotor.getConfigurator()
				.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(120).withSupplyCurrentLimitEnable(false));
	}

	static VelocityVoltage groundMotorVelocityControl = new VelocityVoltage(0).withEnableFOC(true);
	public static void setGroundMotorSpeed(double rps) {
		groundMotor.setControl(groundMotorVelocityControl.withVelocity(rps));
	}

	static VoltageOut groundMotorVoltageControl = new VoltageOut(0).withEnableFOC(true);
	public static void setGroundMotorVoltage(double voltage) {
		groundMotor.setControl(groundMotorVoltageControl.withOutput(voltage));
	}

	public static double getGroundMotorSpeed() {
		groundMotorVelocity.refresh();
		return groundMotorVelocity.getValue();
	}

	static VelocityVoltage kickMotorVelocityControl = new VelocityVoltage(0).withEnableFOC(true);
	public static void setKickMotorSpeed(double rps) {
		kickMotor.setControl(kickMotorVelocityControl.withVelocity(rps));
	}

	static VoltageOut kickMotorVoltageControl = new VoltageOut(0).withEnableFOC(true);
	public static void setKickMotorVoltage(double voltage) {
		kickMotor.setControl(kickMotorVoltageControl.withOutput(voltage));
	}

	public static double getKickMotorSpeed() {
		kickMotorVelocity.refresh();
		return kickMotorVelocity.getValue();
	}

	static VelocityVoltage chamberMotorVelocityControl = new VelocityVoltage(0).withEnableFOC(true);
	public static void setChamberMotorSpeed(double rps) {
		chamberMotor.setControl(chamberMotorVelocityControl.withVelocity(rps));
	}

	static VoltageOut chamberMotorVoltageControl = new VoltageOut(0).withEnableFOC(true);
	public static void setChamberMotorVoltage(double voltage) {
		chamberMotor.setControl(chamberMotorVoltageControl.withOutput(voltage));
	}

	public static double getChamberMotorSpeed() {
		chamberMotorVelocity.refresh();
		return chamberMotorVelocity.getValue();
	}

	public static boolean getChamberSensor() {
		if (IntakeConstants.invertSensors == 1)
			return chamberSensor.get();
		return !chamberSensor.get();
	}

	public static boolean getFrontSensor() {
		if (IntakeConstants.invertSensors == 1)
			return frontSensor.get();
		return !frontSensor.get();
	}

	public static boolean getBackSensor() {
		if (IntakeConstants.invertSensors == 1)
			return backSensor.get();
		return !backSensor.get();
	}

	public static void log() {
		SmartDashboard.putBoolean("/Intake/FrontSensor", getFrontSensor());
		SmartDashboard.putBoolean("/Intake/BackSensor", getBackSensor());
		SmartDashboard.putBoolean("/Intake/ChamberSensor", getChamberSensor());
		SmartDashboard.putNumber("/Intake/GroundRPS", getGroundMotorSpeed());
		SmartDashboard.putNumber("/Intake/KickRPS", getKickMotorSpeed());
		SmartDashboard.putNumber("/Intake/ChamberRPS", getChamberMotorSpeed());
		
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if(ShooterSubsystem.getFeedMotorSpeed() < 1){
		if (ShooterSubsystem.getTargetAngle()> -54) {
			IntakeSubsystem.setChamberMotorSpeed(IntakeConstants.intakeDps / 30);
		} else {
			IntakeSubsystem.setChamberMotorVoltage(0);
		}
	}
	}
}
