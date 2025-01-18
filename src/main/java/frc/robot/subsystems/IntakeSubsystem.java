// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class IntakeSubsystem extends SubsystemBase {
	// Declare
	private TalonFX groundMotor, kickMotor, chamberMotor;
	public IntakeSubsystem() {
		// Initialize
		groundMotor = new TalonFX(IDConstants.groundMotorID, "rio");
		groundMotor.getConfigurator().apply(new TalonFXConfiguration());
		groundMotor.setInverted(true);

		kickMotor = new TalonFX(IDConstants.kickMotorID, "rio");
		kickMotor.getConfigurator().apply(new TalonFXConfiguration());
		kickMotor.setInverted(true);

		chamberMotor = new TalonFX(IDConstants.grabMotorID, "rio");
	}

	public void setgroundMotorVoltage(double voltage) {
		groundMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	public void setkickVoltage(double voltage) {
		kickMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	public void setchamberVoltage(double voltage) {
		chamberMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	// Get/set

	public void log() {
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
