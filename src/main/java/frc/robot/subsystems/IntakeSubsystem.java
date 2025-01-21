// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
	private TalonFX groundMotor, kickMotor, chamberMotor;

	private DigitalInput frontSensor, backSensor, chamberSensor;

	public IntakeSubsystem() {
		groundMotor = new TalonFX(IDConstants.groundMotorID, "rio");
		groundMotor.getConfigurator().apply(new TalonFXConfiguration());
		groundMotor.setInverted(true);
		groundMotor.setNeutralMode(NeutralModeValue.Coast);
		kickMotor = new TalonFX(IDConstants.kickMotorID, "rio");
		kickMotor.getConfigurator().apply(new TalonFXConfiguration());
		kickMotor.setInverted(true);
		kickMotor.setNeutralMode(NeutralModeValue.Coast);

		chamberMotor = new TalonFX(IDConstants.grabMotorID, "rio");

		frontSensor = new DigitalInput(IDConstants.frontSensorChannel);
		backSensor = new DigitalInput(IDConstants.backSensorChannel);
		chamberSensor = new DigitalInput(IDConstants.chamberSensorChannel);
		reloadFromConfig();
	}

	public void reloadFromConfig() {
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

	public void setGroundMotorSpeed(double rps) {
		groundMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
	}

	public void setGroundMotorVoltage(double voltage) {
		groundMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	public double getGroundMotorSpeed() {
		return groundMotor.getVelocity().getValue();
	}

	public void setKickMotorSpeed(double rps) {
		kickMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
	}

	public void setKickMotorVoltage(double voltage) {
		kickMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	public double getKickMotorSpeed() {
		return kickMotor.getVelocity().getValue();
	}

	public void setChamberMotorSpeed(double rps) {
		chamberMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
	}

	public void setChamberMotorVoltage(double voltage) {
		chamberMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	public double getChamberMotorSpeed() {
		return chamberMotor.getVelocity().getValue();
	}

	public boolean getChamberSensor() {
		if (IntakeConstants.invertSensors == 1)
			return chamberSensor.get();
		return !chamberSensor.get();
	}

	public boolean getFrontSensor() {
		if (IntakeConstants.invertSensors == 1)
			return frontSensor.get();
		return !frontSensor.get();
	}

	public boolean getBackSensor() {
		if (IntakeConstants.invertSensors == 1)
			return backSensor.get();
		return !backSensor.get();
	}

	public void log() {
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
	}
}
